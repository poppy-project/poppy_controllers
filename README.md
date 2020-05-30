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
This controller implements the regular robot interface for ROS:

* The topic `/joint_states` publishes the current joint angles and speeds
* The action server `/follow_joint_trajectory` allows to control the robot by executing trajectories of type `trajectory_msgs/JointTrajectory`

## Compatible robots and accessories

Although this ROS package can evolute as a ROS overlay to [`pypot`](https://github.com/poppy-project/pypot) and thus take control over all Poppy robots: Ergo jr, Torso, Humanoid... the current development status currently only supports Poppy Ergo Jr mounted with the Gripper tool. 
If you intend to control other Poppy robots, start by setting the right motors [here](https://github.com/poppy-project/poppy_controllers/blob/69e96dfa1774237e4ae770afbfbc23946c6b7a5f/cfg/JointTrajectoryActionServer.cfg#L65).
