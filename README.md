# ROS Melodic controller for Poppy robots
## Quickstart

This software is distributed as a systme image for the Raspberry Pi so no setup is required except downloading the latest image file and flashing your SD card. 
Start the controller on the Raspberry Pi as follows:
```bash
ssh pi@poppy.local
# Password is "raspberry"

# Make sure your ROS_MASTER_URI variable is correctly set to your usecase:
nano ~/.bashrc

# Then, just run the controllers:
roslaunch poppy_controllers control.launch
```

Once the controller is started, the robot can be driven either via Moveit or by publishing on the ROS API as described hereunder.

## ROS API overview
This controller implements the regular robot interface for ROS + custom services:

* The topic `/joint_states` publishes the current joint angles and speeds
* The action server `/follow_joint_trajectory` allows to control the robot by executing trajectories of type `trajectory_msgs/JointTrajectory`
* The service `/set_compliant` allows to en(dis)able the robot compliance
* The service `/get_image` allows to grab an image from the camera

## Examples

Start the controller hereunder on the Raspberry Pi. Then, our your workstation, try the following:

### 1. Set the robot in compliance mode:
```bash
rosservice call /set_compliant "data: true" 

# The service must return a success message:
#   success: True
#   message: "Robot compliance has been enabled"
```

### 2. Plot the joint positions:
```bash
rosrun rqt_plot rqt_plot /joint_states/position[0] /joint_states/position[1] /joint_states/position[2] /joint_states/position[3] /joint_states/position[4] /joint_states/position[5]
```
You'll see a graph updated in real time. Manually move motors with your hands to see their evolution in real time. 

### 3. Trajectory record and playback feature
```python
import rospy
from poppy_ros_control.recorder import Recorder

rospy.init_node("trajectory_recorder")
r = Recorder()
r.start_recording()

# Move your robot with your hands to record its trajectory

r.stop_and_save("my_motion_name")
```

Trajectories are stored in JSON files in the `poppy_controllers/data` directory. Later, you can replay them this way:
```python
import rospy
from poppy_ros_control.recorder import Player

rospy.init_node("trajectory_player")
p = Player()

my_motion = p.load("my_motion_name")

# load() returns a moveit_msgs/RobotTrajectory object that you can pass to the robot commander:

robot.execute(my_motion)
```

### 4. Grab an image
```python
import cv2
from poppy_controllers.srv import GetImage
from cv_bridge import CvBridge

get_image = rospy.ServiceProxy("get_image", GetImage)
response = get_image()
bridge = CvBridge()
image = bridge.imgmsg_to_cv2(response.image)
cv2.imshow("Poppy camera", image)
cv2.waitKey(200)
```
## Compatible robots and accessories

Although this ROS package can evolute as a ROS overlay to [`pypot`](https://github.com/poppy-project/pypot) and thus take control over all Poppy robots: Ergo jr, Torso, Humanoid... the current development status currently only supports Poppy Ergo Jr mounted with the Gripper tool. 

If you intend to control other Poppy robots, start by setting the right motors [here](https://github.com/poppy-project/poppy_controllers/blob/69e96dfa1774237e4ae770afbfbc23946c6b7a5f/cfg/JointTrajectoryActionServer.cfg#L65).
