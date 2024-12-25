cmake_minimum_required(VERSION 2.8.3)
project(laser_to_arduino)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_to_arduino_node src/laser_to_arduino_node.cpp)
target_link_libraries(laser_to_arduino_node ${catkin_LIBRARIES} serial)
