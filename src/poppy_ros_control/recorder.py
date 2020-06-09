from rospkg import RosPack
from os.path import join, isdir, isfile
from os import makedirs
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import RobotTrajectory
import json, rospy
from copy import deepcopy

def to_dict(points):
    """
    points: list of JointStates: [{"js": JointState, "time": time}, ]
    returns: dict view of the trajectory
    """
    trajectory = {"joints": [], "points": []}
    if len(points) != 0:
        trajectory["joints"] = points[0]["js"].name
        t0 = points[0]["time"]
    
    for point in points:
        if point["js"].name == trajectory["joints"]:
            jtp = {"time": float(point["time"] - t0),
                   "positions": [float(v) for v in point["js"].position]}
            trajectory["points"].append(jtp)
    return trajectory

def from_dict(trajectory):
    """
    trajectory: dict view of a trajectory
    returns: trajectory_msgs/JointTrajectory view of the trajectory
    """
    jt = JointTrajectory()
    jt.joint_names = trajectory["joints"]
    jtp = JointTrajectoryPoint()
    for point in trajectory["points"]:
        jtp.positions = point["positions"]
        jtp.time_from_start = rospy.Duration(point["time"])
        jt.points.append(deepcopy(jtp))
    return jt

class RecorderBase(object):
    MAX_POINTS = 10000

    def __init__(self):
        self._rospack = RosPack()
        self._path = join(self._rospack.get_path("poppy_controllers"), "data")
        self._data = []   

class Recorder(RecorderBase):
    def __init__(self):
        super(Recorder, self).__init__()
        if not isdir(self._path):
            makedirs(self._path)
        self._recording = False
        rospy.Subscriber("joint_states", JointState, self._cb_js)

    def _cb_js(self, js):
        if self._recording:
            if len(self._data) > self.MAX_POINTS:
                self._recording = False
                rospy.logerr("Reached the maximum number of {} JointState points".format(self.MAX_POINTS))
                return
            self._data.append({"js": js, "time": rospy.Time.now().to_sec()})
        
    def start_recording(self):
        if self._recording:
            rospy.logerr("Recording already in progress...")
            return False
        self._recording = True
        return True

    def stop_and_save(self, trajectory_name):
        if not self._recording:
            rospy.logerr("Cannot stop recording because no recording was started")
            return False
        self._recording = False
        file_path = join(self._path, trajectory_name + ".json") 
        if isfile(file_path):
            rospy.logwarn("Overwriting trajectory: '{}'".format(trajectory_name))
        with open(file_path, 'w') as f:
            json.dump(to_dict(self._data), f)
        self._data = []
        return True


class Player(RecorderBase):   
    def load(self, trajectory_name):       
        file_path = join(self._path, trajectory_name + ".json") 
        if not isfile(file_path):
            rospy.logerr("This trajectory does not exist: '{}'".format(trajectory_name))
            return None
        
        with open(file_path) as f:
            data = json.load(f)
            jt = from_dict(data)
            return RobotTrajectory(joint_trajectory=jt)
