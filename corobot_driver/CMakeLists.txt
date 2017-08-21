cmake_minimum_required(VERSION 2.8.3)
project(corobot_driver)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  phidgets_api
  hardware_interface
  controller_manager
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(corobot_base_driver src/corobot_driver.cpp)
add_dependencies(corbot_base_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(corobot_base_driver
  ${catkin_LIBRARIES}
)

