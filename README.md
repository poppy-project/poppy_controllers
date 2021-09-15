# ROS Noetic controller for Poppy robots
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
* The service `/close_gripper` allows to close or open the gripper (motor m6 of the Ergo Jr)
* The parameters `/gripper/angles/aperture` and `/gripper/angles/closure` define the range of aperture of the gripper (in degrees from about -20° to +30°)
* The parameter `/gripper/speed` defines the opening/closing speed from 0.05 (slowest) to 1 (fastest)

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

### 3. Plan and execute trajectories with MoveIt
Use this package with [poppy_ergo_jr_moveit_config](https://github.com/poppy-project/poppy_ergo_jr_moveit_config) in order to plan collision-free trajectories and execute them on the robot.

### 4. Trajectory record and playback feature
To record a trajectory, it is handy to enable compliance and manipulate the robot with your hands: 
```bash
rosservice call /set_compliant "data: true" 
```
However, it is also totally fine to record a trajectory played by MoveIt or any other way.

Then, open a Jupyter notebook or an `ipython` interactive interpreter and use the following functions to record, for instance, a 5sec trajectory:
```python
import rospy
from poppy_ros_control.recorder import Recorder

rospy.init_node("trajectory_recorder")
r = Recorder()
r.start_recording()

# Move your robot with your hands in compliant mode to record its trajectory
rospy.sleep(5)

r.stop_and_save("my_motion_name")
```

Trajectories are stored in JSON files in the `poppy_controllers/data` directory.

Later, you can replay them this way:


```python
import rospy
from poppy_ros_control.recorder import Player
from moveit_commander.move_group import MoveGroupCommander

rospy.init_node("trajectory_player")
commander = MoveGroupCommander("arm_and_finger")
player = Player()

# This returns a moveit_msgs/RobotTrajectory object representing the recorded trajectory
my_motion = player.load("my_motion_name")

# Go to the start position before replaying the motion
commander.set_joint_value_target(my_motion.joint_trajectory.points[0].positions)
commander.go()

# Replay the exact same motion
commander.execute(my_motion)
```

### 5. Grab an image
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
## Troubleshooting
#### `Invalid Trajectory: start point deviates from current robot state more than 0.2`
You're probably trying to replay a trajectory while your robot didn't reach the starting point first. Make sure you reach it with `set_joint_value_target`.

#### `ABORTED: Solution found but controller failed during execution`
Is your robot compliance disabled? No trajectory can be executed with compliance.

#### Robot makes abrupt trajectories
* If you are replaying a recorded trajectory, make you you first join its initial point before starting replay: use `set_joint_value_target` first before `execute`
* Poppy Ergo Jr's motors have a range of [-170°, +170°] = [-0.94 rad, +0.94 rad], if your trajectories don't fit this interval, you will likely have erratic movements, thus:
* keep away from U-turns (~ 180° = 3.14 rad) for each motor when recording a trajectory
* make sure your motors are not mounted backwards : `set_joint_value_target([0, 0, 0, 0, 0, 0])` must bring your robot in [that exact configuration](https://camo.githubusercontent.com/bda29f64b2e37ca0471eefff12f7981300e167c8/687474703a2f2f646f63732e706f7070792d70726f6a6563742e6f72672f656e2f617373656d626c792d6775696465732f6572676f2d6a722f696d672f6572676f5f746f6f6c732e676966).


## Compatible robots and accessories

Although this ROS package can evolute as a ROS overlay to [`pypot`](https://github.com/poppy-project/pypot) and thus take control over all Poppy robots: Ergo jr, Torso, Humanoid... the current development status currently only supports Poppy Ergo Jr mounted with the Gripper or Lamp tools. 

If you intend to control other Poppy robots, start by setting the right motors [here](https://github.com/poppy-project/poppy_controllers/blob/69e96dfa1774237e4ae770afbfbc23946c6b7a5f/cfg/JointTrajectoryActionServer.cfg#L65).
