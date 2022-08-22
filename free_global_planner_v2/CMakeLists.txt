cmake_minimum_required(VERSION 3.0.2)
project(free_global_planner_v2)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(free_global_planner_v2_lib src/free_planner.cpp)
