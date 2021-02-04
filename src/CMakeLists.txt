cmake_minimum_required(VERSION 3.5)
project(Main)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(AGV)

add_executable(agv main.cpp)
target_link_libraries(agv AGV)