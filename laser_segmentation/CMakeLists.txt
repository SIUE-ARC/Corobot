cmake_minimum_required(VERSION 2.8.3)
project(laser_segmentation)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  tf2
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_segmentation src/laser_segmentation.cpp)
add_dependencies(laser_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(laser_segmentation
  ${catkin_LIBRARIES}
)

