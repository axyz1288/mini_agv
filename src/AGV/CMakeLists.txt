cmake_minimum_required(VERSION 3.5)
project(AGV)

find_package(slamware_ros_sdk)

add_subdirectory(Car)

add_library(AGV AGV.cpp)
target_link_libraries(AGV Car)
target_include_directories(Car PUBLIC ${slamware_ros_sdk_INCLUDE_DIRS})