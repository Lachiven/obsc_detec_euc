cmake_minimum_required(VERSION 2.8.3)
project(obsc)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS})

 catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros
  DEPENDS PCL )

 add_executable(obj_detection_node src/obj_detection.cpp)
 target_link_libraries(obj_detection_node  ${catkin_LIBRARIES})
