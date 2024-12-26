cmake_minimum_required(VERSION 3.0.2)
project(laser_to_arduino)

# 使用 C++11 标准
add_compile_options(-std=c++11)

# 查找 catkin 所需组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
)

# 声明 catkin 包
catkin_package()

# 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(${PROJECT_NAME} src/laser_to_arduino.cpp)

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
