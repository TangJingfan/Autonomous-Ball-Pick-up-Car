cmake_minimum_required(VERSION 3.0.2)
project(laser_odom)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

## System dependencies
find_package(Eigen3 REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


################################################
## catkin specific configuration ##
###########################################
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf geometry_msgs pcl_ros pcl_conversions
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/nanoflann/include  # Corrected path to nanoflann
)

## Declare a C++ executable
add_executable(laser_odom_node src/laser_odom.cpp)

## Link libraries
target_link_libraries(laser_odom_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  pcl_ros
  pcl_conversions
)

#############
## Install ##
#############
# If you want to install your executable:
# install(TARGETS icp_localization_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

# Add gtest based cpp test target if you need
# catkin_add_gtest(${PROJECT_NAME}-test test/test_icp_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
