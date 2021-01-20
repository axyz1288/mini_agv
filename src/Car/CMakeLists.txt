cmake_minimum_required(VERSION 3.5)
project(Car)

add_subdirectory(MotorUnion)

find_package(catkin REQUIRED COMPONENTS nodelet)

add_library(Car Car.cpp)
target_link_libraries(Car MotorUnion ${catkin_LIBRARIES})
target_include_directories(Car PUBLIC ${catkin_INCLUDE_DIRS})