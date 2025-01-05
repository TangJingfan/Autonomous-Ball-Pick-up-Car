cmake_minimum_required(VERSION 3.0.2)
project(pgm_to_pointcloud)

## 编译选项
add_compile_options(-std=c++11)

## 查找catkin和系统依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

## 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## 定义catkin_package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

## 声明可执行文件
add_executable(${PROJECT_NAME}_node src/pgm_to_pointcloud.cpp)

## 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

## 安装目标
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
