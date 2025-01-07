cmake_minimum_required(VERSION 3.10)  # 设置CMake的最低版本要求

project(AStarProject)  # 项目名称

# 找到 OpenCV 库
find_package(OpenCV REQUIRED)

# 将源代码文件添加到项目中
add_executable(a_star src/a_star.cpp)

# 链接 OpenCV 库
target_link_libraries(a_star ${OpenCV_LIBS})
