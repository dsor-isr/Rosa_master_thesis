cmake_minimum_required(VERSION 3.1.0)
project(surge_controller)

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
  ChangeSurgeInnerGains.srv
  ChangeSurgeFFGains.srv
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
 include/surge_controller_ros
 include/surge_controller_algorithms
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/surge_controller_ros/SurgeControllerNode.cpp src/surge_controller_algorithms/SurgeControllerAlgorithm.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
