#include <iostream>
#include <string>

#include "routing_problem.hpp"

using namespace routing;

int main(int argc, char const *argv[])
{
	const std::string path = "../sample/problem.txt";

	RoutingProblem problem(path);

	return 0;
}
