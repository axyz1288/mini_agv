cmake_minimum_required(VERSION 3.5)
project(AGV)

add_subdirectory(MotorUnion)

find_package(catkin REQUIRED COMPONENTS nodelet)

add_library(AGV AGV.cpp)
target_link_libraries(AGV MotorUnion ${catkin_LIBRARIES})
target_include_directories(AGV PUBLIC ${catkin_INCLUDE_DIRS})
