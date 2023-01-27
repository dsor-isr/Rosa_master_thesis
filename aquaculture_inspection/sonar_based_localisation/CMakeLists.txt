cmake_minimum_required(VERSION 3.1.0)
project(sonar_based_localisation)

find_package(catkin 
  # ROS components
  REQUIRED COMPONENTS
  std_msgs
  farol_msgs
  rospy
  message_generation
)

catkin_python_setup()


add_message_files(
    FILES
    DetectionResults.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
)
