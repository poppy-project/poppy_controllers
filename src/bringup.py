#!/usr/bin/env python3
import rospy
from pypot.primitive.move import MoveRecorder, Move, MovePlayer
from poppy_ergo_jr import PoppyErgoJr
from poppy_controllers.srv import GetImage, GetImageResponse
from std_srvs.srv import SetBool, SetBoolResponse


class RecordReplayBringupServer:
    def __init__(self):
        self._recording = ""
        # Actual robot control
        self._cv_bridge = None
        self._robot = None

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

    def _cb_get_image(self, request):
        if self._cv_bridge is None:
            rospy.logerr("Camera image requested, but no CvBridge available, returning an empty image")
            return GetImageResponse()
        else:
            image = self._robot.camera.frame
            rospy.loginfo("Camera image requested, returning an image of size", image.size)
            return GetImageResponse(self._cv_bridge.cv2_to_imgmsg(image))

    def _cb_set_compliant(self, request):
        self._robot.compliant = request.data
        msg = "Robot compliance has been {}".format('enabled' if request.data else 'disabled')
        return SetBoolResponse(success=True, message=msg)

    def connect(self):
        try:
            self._robot = PoppyErgoJr()
        except OSError:
            rospy.logwarn("Can't connect to the robot, let's disable the camera...")
            try:
                self._robot = PoppyErgoJr(camera='dummy')
            except OSError as e:
                rospy.logwarn("Connection to the robot can't be established:")
                rospy.logwarn(str(e))
                self._robot = None
            else:
                rospy.logwarn("Connection successful but camera was disabled")

        if self._robot is not None:
            from cv_bridge import CvBridge
            self._cv_bridge = CvBridge()
    
            # Start the robot
            for m in self._robot.motors:
                m.moving_speed = 250
            self._robot.compliant = False
            self._recorder = MoveRecorder(self._robot, rospy.get_param("recorder/rate", 20), self._robot.motors)

            # Start services
            self._compliant_srv = rospy.Service('set_compliant', SetBool, self._cb_set_compliant)
            self._image_srv = rospy.Service('get_image', GetImage, self._cb_get_image)
            self._gripper_service = rospy.Service('close_gripper', SetBool, self._cb_close_gripper)   
            return True
        return False 

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            record = rospy.get_param("recorder/record", None)
            if record is None:
                rospy.set_param("recorder/record", "")
            elif record == "" and self._recording != "":
                # Stop recording
                self._recorder.stop()
                rospy.loginfo("Recording stopped")
                with open(self._recording+'.json', 'w') as f:
                    self._recorder.move.save(f)
                self._recording = ""
            elif record != "" and self._recording == "":
                # Start recording
                self._recorder.start()
                rospy.logwarn("Started recording motion named '{}'".format(record))
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("poppy_bringup")
    node = RecordReplayBringupServer()
    connected = node.connect()
    if connected: node.run()
