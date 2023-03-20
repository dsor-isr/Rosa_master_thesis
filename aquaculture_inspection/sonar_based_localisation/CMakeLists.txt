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

#rosbuild_gensrv()

add_service_files(
  FILES
  ChangeSSEValues.srv
  ChangeDistanceParameters.srv
  ChangeNetRadius.srv
)

add_message_files(
    FILES
    DetectionResults.msg
    FloatArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
)
