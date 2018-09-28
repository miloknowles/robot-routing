############# CONFIGURATION ##############

cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(robot_routing LANGUAGES CXX VERSION 0.0.0.0)

option(BUILD_SHARED_LIBS "Build this project as a SHARED library instead of STATIC." OFF)

# Compile with c++11.
set(CMAKE_CXX_STANDARD 11)

############# GOOGLE TEST ###############
# Download and unpack googletest at configure time.
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

############### ROBOT ROUTING ###################

# Add the robot routing library.
add_library(robot_routing
	utils.cpp
	routing_problem.cpp
)

# Add the main solver executable.
add_executable(run_solver solver.cpp)
target_link_libraries(run_solver
	robot_routing)

# Add tests and link against gtest.
add_executable(run_tests test.cpp)
target_link_libraries(run_tests
	robot_routing
	gtest_main)
