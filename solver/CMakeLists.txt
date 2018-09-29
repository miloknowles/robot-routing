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
