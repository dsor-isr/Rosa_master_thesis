cmake_minimum_required(VERSION 3.1.0)
project(detection)

find_package(catkin 
  # ROS components
  REQUIRED COMPONENTS
  std_msgs
  farol_msgs
  rospy
  cv_bridge
  auv_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)
