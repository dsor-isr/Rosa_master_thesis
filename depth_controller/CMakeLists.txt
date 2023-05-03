cmake_minimum_required(VERSION 3.1.0)
project(depth_controller)

find_package(catkin
 # ROS components
 REQUIRED COMPONENTS
 std_msgs
 farol_msgs
 dsor_utils
 dsor_msgs
 roscpp
 auv_msgs
 farol_gimmicks_library
 message_generation
)

add_compile_options(-std=c++17 -Wall -O3) 

add_service_files(
  FILES
  ChangeDepthInnerGains.srv
  ChangeDepthFFGains.srv
  ChangeKffLamb.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
 CATKIN_DEPENDS
 dsor_utils 
 dsor_msgs 
 roscpp 
 farol_gimmicks_library 
 auv_msgs
)

include_directories(
 include/depth_controller_ros
 include/depth_controller_algorithms
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/depth_controller_ros/DepthControllerNode.cpp src/depth_controller_algorithms/DepthControllerAlgorithm.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
