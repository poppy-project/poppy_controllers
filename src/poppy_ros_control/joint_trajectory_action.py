# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
# JTAS adapted by Yoan Mollard for Poppy robots, meeting the license hereunder
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import errno
import actionlib
import bisect
from copy import deepcopy
import operator
import numpy as np
import poppy_ros_control.bezier as bezier
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import UInt16
from poppy_controllers.srv import GetImage, GetImageResponse
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import JointState
from poppy_ergo_jr import PoppyErgoJr
from rospy import ROSException

DEG_TO_RAD = 0.0174527


class JointTrajectoryActionServer(object):
    def __init__(self, reconfig_server, rate=100.0):
        self._dyn = reconfig_server
        self.continuous = self._dyn.config['continuous']
        self._fjt = '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._fjt,
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)
        self._action_name = rospy.get_name()

        # Current joint states
        self._js = JointState()
        self._js_publisher = rospy.Publisher('joint_states', JointState, queue_size=10)

        # Actual robot control
        self._cv_bridge = None
        try:
            self._robot = PoppyErgoJr()
        except OSError:
            rospy.logwarn("Can't connect to the robot, let's disable the camera...")
            try:
                self._robot = PoppyErgoJr(camera='dummy')
            except OSError as e:
                rospy.logwarn("Connection to the robot can't be established:" + str(e))
                self._robot = None
                return
            else:
                rospy.logwarn("Connection successful but camera was disabled")
        else:
            from cv_bridge import CvBridge
            self._cv_bridge = CvBridge()

        self._motors = [m.name for m in self._robot.motors]
        self._js.name = deepcopy(self._motors)

        # Action Feedback/Result
        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()

        # Controller parameters from arguments, messages, and dynamic reconfigure
        self._control_rate = rate  # Hz
        self._update_rate_spinner = rospy.Rate(self._control_rate)
        self._control_joints = []
        self._goal_time = 0.0
        self._stopped_velocity = 0.0
        self._goal_error = dict()
        self._path_thresh = dict()

        # Start the robot
        for m in self._robot.motors:
            m.moving_speed = 250
        self._robot.compliant = False

        # Start services
        self._compliant_srv = rospy.Service('set_compliant', SetBool, self._cb_set_compliant)
        self._image_srv = rospy.Service('get_image', GetImage, self._cb_get_image)
        self._gripper_service = rospy.Service('close_gripper', SetBool, self._cb_close_gripper)
        # Start the action server
        rospy.sleep(0.1)
        self._server.start()

    def _cb_close_gripper(self, req):
        # Only for Poppy Ergo Jr with the gripper end effector (motor m6)
        msg="Gripper is going to {}".format("open" if not req.data else "close")
        success = True
        target_open = min(60, rospy.get_param("gripper/angles/aperture", 30))     # Gripper aperture angle in degrees
        target_close = max(-45, rospy.get_param("gripper/angles/closure", -20))   # Gripper closure angle in degrees
        speed = min(1, max(0.05, rospy.get_param("gripper/speed", 0.2)))      # Gripper opening/closure speed from 0.05 to 1
        rospy.set_param("gripper/angles/aperture", target_open)
        rospy.set_param("gripper/angles/closure", target_close)
        rospy.set_param("gripper/speed", speed)
        try:
            self._robot.m6.goto_position(target_close if req.data else target_open, 0.1/speed, wait=False)
        except AttributeError:
            msg = "Gripper opening and closure only works with motor m6 of Poppy Ergo Jr but no such motor is available"
            success = False
        return SetBoolResponse(success=success, message=msg)

    def spin(self):
        while not rospy.is_shutdown():
            self._js.header.stamp = rospy.Time.now()
            self._js.position = [getattr(self._robot, m).present_position*DEG_TO_RAD for m in self._motors]
            self._js.velocity = [getattr(self._robot, m).present_speed*DEG_TO_RAD for m in self._motors]
            try:
                self._js_publisher.publish(self._js)
            except ROSException:
                pass
            self._update_rate_spinner.sleep()

    def clean_shutdown(self):
        self._robot.compliant = True   # Stop the robot
        rospy.sleep(0.5)

    def _cb_get_image(self, request):
        if self._cv_bridge is None:
            return GetImageResponse()
        else:
            image = self._robot.camera.frame
            return GetImageResponse(self._cv_bridge.cv2_to_imgmsg(image))

    def _cb_set_compliant(self, request):
        self._robot.compliant = request.data
        msg = "Robot compliance has been {}".format('enabled' if request.data else 'disabled')
        return SetBoolResponse(success=True, message=msg)

    def _get_trajectory_parameters(self, joint_names, goal):
        # For each input trajectory, if path, goal, or goal_time tolerances
        # provided, we will use these as opposed to reading from the
        # parameter server/dynamic reconfigure

        # Goal time tolerance - time buffer allowing goal constraints to be met
        if goal.goal_time_tolerance:
            self._goal_time = goal.goal_time_tolerance.to_sec()
        else:
            self._goal_time = self._dyn.config['goal_time']
        # Stopped velocity tolerance - max velocity at end of execution
        self._stopped_velocity = self._dyn.config['stopped_velocity_tolerance']

        # Path execution and goal tolerances per joint
        for jnt in joint_names:
            if jnt not in self._motors:
                rospy.logerr("%s: Trajectory Aborted - Provided Invalid Joint Name %s" % (self._action_name, jnt,))
                self._result.error_code = self._result.INVALID_JOINTS
                self._server.set_aborted(self._result)
                return
            # Path execution tolerance
            path_error = self._dyn.config[jnt + '_trajectory']
            if goal.path_tolerance:
                for tolerance in goal.path_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._path_thresh[jnt] = tolerance.position
                        else:
                            self._path_thresh[jnt] = path_error
            else:
                self._path_thresh[jnt] = path_error
            # Goal error tolerance
            goal_error = self._dyn.config[jnt + '_goal']
            if goal.goal_tolerance:
                for tolerance in goal.goal_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._goal_error[jnt] = tolerance.position
                        else:
                            self._goal_error[jnt] = goal_error
            else:
                self._goal_error[jnt] = goal_error

    def _get_current_position(self, joint_names):
        return [getattr(self._robot, joint).present_position/DEG_TO_RAD for joint in joint_names]

    def _get_current_velocities(self, joint_names):
        return [getattr(self._robot, joint).present_speed/DEG_TO_RAD for joint in joint_names]

    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position(joint_names)
        error = list(map(operator.sub, set_point, current))
        return zip(joint_names, error)

    def _update_feedback(self, cmd_point, jnt_names, cur_time):
        cur_time = rospy.Duration(cur_time)
        self._fdbk.header.stamp = rospy.Time.now()
        self._fdbk.joint_names = jnt_names
        self._fdbk.desired = cmd_point
        self._fdbk.desired.time_from_start = cur_time
        self._fdbk.actual.positions = self._get_current_position(jnt_names)
        self._fdbk.actual.time_from_start = cur_time
        self._fdbk.error.positions = list(map(operator.sub,
                                         self._fdbk.desired.positions,
                                         self._fdbk.actual.positions))
        self._fdbk.error.time_from_start = cur_time


        self._server.publish_feedback(self._fdbk)

    def _command_joints(self, joint_names, point):

        # point in a trajectory_msgs/JointTrajectoryPoint
        # We're controlling motors in position at pypot's default max speed
        if len(joint_names) != len(point.positions):
            rospy.logerr("Point is invalid: len(joint_names) != len(point.positions)")
            return False
        for i, m in enumerate(joint_names):
            if m not in self._motors:
                rospy.logerr("Point is invalid: joint {} not found".format(m))
                return False
            if self._robot.compliant:
                rospy.logerr("Robot is compliant, the trajectory cannot execute")
                return False
            dxl = getattr(self._robot, m)
            dxl.goal_position = float(point.positions[i])/DEG_TO_RAD
        return True

    def _determine_dimensions(self, trajectory_points):
        # Determine dimensions supplied
        position_flag = True
        velocity_flag = (len(trajectory_points[0].velocities) != 0 and
                         len(trajectory_points[-1].velocities) != 0)
        acceleration_flag = (len(trajectory_points[0].accelerations) != 0 and
                             len(trajectory_points[-1].accelerations) != 0)
        return {'positions': position_flag,
                'velocities': velocity_flag,
                'accelerations': acceleration_flag}

    def _compute_bezier_coeff(self, joint_names, trajectory_points, dimensions_dict):
        # Compute Full Bezier Curve
        num_joints = len(joint_names)
        num_traj_pts = len(trajectory_points)
        num_traj_dim = sum(dimensions_dict.values())
        num_b_values = len(['b0', 'b1', 'b2', 'b3'])
        b_matrix = np.zeros(shape=(num_joints, num_traj_dim, num_traj_pts-1, num_b_values))
        for jnt in range(num_joints):
            traj_array = np.zeros(shape=(len(trajectory_points), num_traj_dim))
            for idx, point in enumerate(trajectory_points):
                current_point = list()
                current_point.append(point.positions[jnt])
                if dimensions_dict['velocities']:
                    current_point.append(point.velocities[jnt])
                if dimensions_dict['accelerations']:
                    current_point.append(point.accelerations[jnt])
                traj_array[idx, :] = current_point
            d_pts = bezier.de_boor_control_pts(traj_array)
            b_matrix[jnt, :, :, :] = bezier.bezier_coefficients(traj_array, d_pts)
        return b_matrix

    def _get_bezier_point(self, b_matrix, idx, t, cmd_time, dimensions_dict):
        pnt = JointTrajectoryPoint()
        pnt.time_from_start = rospy.Duration(cmd_time)
        num_joints = b_matrix.shape[0]
        pnt.positions = [0.0] * num_joints
        if dimensions_dict['velocities']:
            pnt.velocities = [0.0] * num_joints
        if dimensions_dict['accelerations']:
            pnt.accelerations = [0.0] * num_joints
        for jnt in range(num_joints):
            b_point = bezier.bezier_point(b_matrix[jnt, :, :, :], idx, t)
            # Positions at specified time
            pnt.positions[jnt] = b_point[0]
            # Velocities at specified time
            if dimensions_dict['velocities']:
                pnt.velocities[jnt] = b_point[1]
            # Accelerations at specified time
            if dimensions_dict['accelerations']:
                pnt.accelerations[jnt] = b_point[-1]
        return pnt

    def _on_trajectory_action(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        dimensions_dict = self._determine_dimensions(trajectory_points)

        # Load parameters for trajectory
        self._get_trajectory_parameters(joint_names, goal)
        # Create a new discretized joint trajectory
        num_points = len(trajectory_points)
        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
            self._server.set_aborted()
            return
        rospy.logwarn("{}: Executing requested joint trajectory {:0.1f} sec".format(self._action_name, trajectory_points[-1].time_from_start.to_sec()))
        control_rate = rospy.Rate(self._control_rate)

        if num_points == 1:
            # Add current position as trajectory point
            first_trajectory_point = JointTrajectoryPoint()
            first_trajectory_point.positions = self._get_current_position(joint_names)
            # To preserve desired velocities and accelerations, copy them to the first
            # trajectory point if the trajectory is only 1 point.
            if dimensions_dict['velocities']:
                first_trajectory_point.velocities = deepcopy(trajectory_points[0].velocities)
            if dimensions_dict['accelerations']:
                first_trajectory_point.accelerations = deepcopy(trajectory_points[0].accelerations)
            first_trajectory_point.velocities = deepcopy(trajectory_points[0].velocities)
            first_trajectory_point.accelerations = deepcopy(trajectory_points[0].accelerations)
            first_trajectory_point.time_from_start = rospy.Duration(0)
            trajectory_points.insert(0, first_trajectory_point)
            num_points = len(trajectory_points)

        if not self.continuous:
            if dimensions_dict['velocities']:
                trajectory_points[-1].velocities = [0.0] * len(joint_names)
            if dimensions_dict['accelerations']:
                trajectory_points[-1].accelerations = [0.0] * len(joint_names)

        # Compute Full Bezier Curve Coefficients for all 7 joints
        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]
        try:
            b_matrix = self._compute_bezier_coeff(joint_names, trajectory_points, dimensions_dict)
        except Exception as ex:
            rospy.logerr("Failed to compute a Bezier trajectory: {}".format(repr(ex)))
            self._server.set_aborted()
            return

        # Loop until end of trajectory time.  Provide a single time step
        # of the control rate past the end to ensure we get to the end.
        # Keep track of current indices for spline segment generation
        start_time = rospy.Time.now().to_sec()
        now_from_start = 0
        end_time = trajectory_points[-1].time_from_start.to_sec()

        while now_from_start < end_time and not rospy.is_shutdown():
            # Acquire Mutex
            now = rospy.Time.now().to_sec()
            now_from_start = now - start_time
            idx = bisect.bisect(pnt_times, now_from_start)
            # Calculate percentage of time passed in this interval
            if idx >= num_points:
                cmd_time = now_from_start - pnt_times[-1]
                t = 1.0
            elif idx >= 0:
                cmd_time = now_from_start - pnt_times[idx-1]
                t = cmd_time / max(0.001, pnt_times[idx] - pnt_times[idx-1])
            else:
                cmd_time = 0
                t = 0

            point = self._get_bezier_point(b_matrix, idx, t, cmd_time, dimensions_dict)

            # Command Joint Position, Velocity, Acceleration
            command_executed = self._command_joints(joint_names, point)
            self._update_feedback(deepcopy(point), joint_names, now_from_start)
            if not command_executed:
                return
            control_rate.sleep()

        # Keep trying to meet goal until goal_time constraint expired
        last = trajectory_points[-1]
        last_time = trajectory_points[-1].time_from_start.to_sec()

        while now_from_start < last_time + self._goal_time and not rospy.is_shutdown():
            if not self._command_joints(joint_names, last):
                self._server.set_aborted(self._result)
                rospy.logwarn("%s: Action Aborted" % (self._action_name,))
                return
            now_from_start = rospy.Time.now().to_sec() - start_time
            self._update_feedback(deepcopy(last), joint_names, now_from_start)
            control_rate.sleep()

        rospy.loginfo("%s: Joint Trajectory Action Succeeded" % (self._action_name,))
        self._result.error_code = self._result.SUCCESSFUL
        self._server.set_succeeded(self._result)

