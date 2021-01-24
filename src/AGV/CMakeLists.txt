cmake_minimum_required(VERSION 3.5)
project(AGV)

add_subdirectory(Car)

add_library(AGV AGV.cpp)
target_link_libraries(AGV Car)