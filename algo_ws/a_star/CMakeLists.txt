cmake_minimum_required(VERSION 3.10)  

project(AStarProject) 


find_package(OpenCV REQUIRED)


add_executable(a_star src/a_star.cpp)


target_link_libraries(a_star ${OpenCV_LIBS})


set(CMAKE_CXX_FLAGS_DEBUG "-g")

