cmake_minimum_required(VERSION 3.0.2)
project(laser_to_arduino)

# use c++11 standard to compile
add_compile_options(-std=c++11)

# find necessary component
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
)

# declare catkin package
catkin_package()

# include header
include_directories(
  ${catkin_INCLUDE_DIRS}
  src/include
)

# add executable
add_executable(${PROJECT_NAME} 
src/laser_to_arduino.cpp 
src/exploration_rtt.cpp)

# link libraries
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
