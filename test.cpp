#include <gtest/gtest.h>

#include "routing_problem.hpp"

using namespace routing;

/**
 * @brief Make sure that a problem is loaded in properly.
 */
TEST(RoutingProblemTest, TestConstructor)
{
	const std::string problem_path("../sample/problem.txt");
	std::cout << "Loading from: " << problem_path << std::endl;
	RoutingProblem problem(problem_path);
}

// Run all the tests that were declared with TEST().
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
