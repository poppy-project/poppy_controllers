cmake_minimum_required(VERSION 2.8.3)
project(poppy_controllers)

find_package(catkin REQUIRED
  COMPONENTS
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/JointTrajectoryActionServer.cfg
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  message_runtime
  dynamic_reconfigure
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})