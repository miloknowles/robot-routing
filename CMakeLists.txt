cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

# Compile with c++11.
set(CMAKE_CXX_STANDARD 11)

add_library(robot_routing
	utils.cpp
	routing_problem.cpp
)
