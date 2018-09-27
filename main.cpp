#include <iostream>
#include <string>

#include "routing_problem.hpp"

using namespace routing;

int main(int argc, char const *argv[])
{
	// Load in a problem.txt from CLI.
	if (argc < 2) {
		throw std::runtime_error("Need to specify a filename for a problem.txt");
	}
	
	const std::string problem_path(argv[1]);
	std::cout << "Loading from: " << problem_path << std::endl;

	// Set up the routing problem.
	RoutingProblem problem(problem_path);

	return 0;
}
