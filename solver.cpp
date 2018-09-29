#include <iostream>
#include <queue>
#include <algorithm>
#include <string>
#include <tuple>
#include <unordered_set>
#include <chrono>

#include "routing_problem.hpp"
#include "utils.hpp"

using namespace routing;

static constexpr int kTotalUniqueTimesteps = 12;

/**
 * @brief Custom comparator for nodes, used in priority queue.
 * Note: greater than is used to make this a min queue.
 */
class CompareNode {
 public:
  bool operator()(const Node& lhs, const Node& rhs)
  {
  	return (lhs.cost_using_node > rhs.cost_using_node);
  }
};

/**
 * @brief A priority queue of search nodes is used to perform best-first search.
 */
typedef std::priority_queue<Node, std::vector<Node>, CompareNode> PriorityQueue;

/**
 * @brief Returns the estimated cost-to-go from a node.
 * 
 * Note: this must be admissible in order for A* optimality. If there were no wormholes, this could
 * simply be the Manhattan distance between a node and the goal, since that would be a lower bound
 * on the distance left. However, the wormholes create shortcuts that break this heuristic.
 * One possible heuristic is to consider all possible ways to use wormholes to get to the goal,
 * but this becomes exponential in the number of wormholes.
 * 
 * Setting the heuristic to zero will simply perform a breadth-first search, since all nodes with
 * equal cost-so-far will be given equal priority.
 */
float HeuristicCostEstimate(const RoutingProblem& problem, const Node& node)
{
	// const Point2i& goal_point = problem.GoalPoint();

	// // Manhattan distance (if no obstacles were present).
	// float min_dist = ManhattanDistance(node.point, goal_point);

	// // Need to consider shortcuts due to wormholes appearing.
	// // This ensures that the heuristic is admissible (does not overestimate distance).
	// const int sec_until_wormhole = 3 - (node.timestep % 3);

	// for (const Wormhole& wh : problem.Wormholes()) {
	// 	// Could go through the wormhole in either direction.
	// 	const float d1 = ManhattanDistance(node.point, wh.first) + ManhattanDistance(wh.second, goal_point);
	// 	const float d2 = ManhattanDistance(node.point, wh.second) + ManhattanDistance(wh.first, goal_point);

	// 	// Need to move around for sec_until_wormhole in addition to the distance.
	// 	const float wormhole_dist = std::min(d1, d2) + sec_until_wormhole;
	// 	min_dist = std::min(min_dist, wormhole_dist);
	// }

	// return min_dist;
	return 0.0f;
}

/**
 * @brief Trace back parent pointers to reconstruct a path from start to goal.
 * @param[in] nodes All nodes explored during the search. Each node will have a parent index that
 * points to another node in nodes.
 * @param[in] from_idx The index in nodes of the goal nodes (where to start retracing). This should
 * normally be the last index of nodes.
 */
std::vector<Point2i> ReconstructPath(const std::vector<Node>& nodes, const int from_idx)
{
	// Expect from_idx to point to the goal node (should be the last).
	int current_idx = from_idx;
	std::vector<Point2i> path;

	while (current_idx >= 0) {
		path.emplace_back(nodes.at(current_idx).point);
		current_idx = nodes.at(current_idx).parent;
	}

	// Reverse the path to put in correct order.
	std::reverse(path.begin(), path.end());

	return path;
}

/**
 * @brief Perform best-first search to find a shortest path from start to goal.
 * With no heuristic, this is equivalent to BFS. If an admissible heuristic is
 * provided, this will become A* search.
 */
bool FindShortestPath(const RoutingProblem& problem, std::vector<Point2i>* path)
{
	// Maintain a 3D grid of points in the search space that have been added to
	// the priority queue. This helps avoid revisiting points in search space.
	// See APPENDIX.md for an explanation of where the constant 12 comes from.
	const std::pair<size_t, size_t>& dims = problem.ObstacleMap()->Dimensions();
	const Point2i& min_cell = problem.ObstacleMap()->MinCellCoord();
	Grid<std::vector<bool>> expanded(dims.first, dims.second, min_cell,
																	 std::vector<bool>(kTotalUniqueTimesteps, 0));

	PriorityQueue pq;

	// Each time a node is expanded, it's stored here for path reconstruction later.
	std::vector<Node> expanded_nodes;

	// Add the start node: parent = -1 by default.
	Node start_node(0, problem.OriginPoint());
	start_node.cost_using_node = HeuristicCostEstimate(problem, start_node);
	pq.push(start_node);

	while (!pq.empty()) {
		// Expand the node with minimum cost_using_node.
		const Node best = pq.top();
		expanded_nodes.emplace_back(best);
		pq.pop();

		// If goal is found, retrace the path and return success.
		if (best.point == problem.GoalPoint()) {
			*path = ReconstructPath(expanded_nodes, expanded_nodes.size()-1);
			return true;
		}

		// Add valid neighboring nodes.
		std::vector<Node> neighbors = problem.GetNeighbors(best);

		for (Node& neighbor : neighbors) {
			// Skip this neighbor if an equivalent point in the search space has already been explored.
			if (expanded.GetCell(neighbor.point).at(neighbor.timestep % kTotalUniqueTimesteps) == true) {
				continue;
			}

			// (Estimated) total cost using node = cost-so-far + cost-to-go
			neighbor.cost_using_node = neighbor.timestep + HeuristicCostEstimate(problem, neighbor);

			// Point this node to the parent that was just expanded (and added to expanded_nodes).
			neighbor.parent = (expanded_nodes.size() - 1);
			pq.push(neighbor);

			// Mark that this search point (x, y, t % 12) has been pushed to the queue.
			std::vector<bool>* const expanded_point = expanded.GetCellMutable(neighbor.point);
			expanded_point->at(neighbor.timestep % kTotalUniqueTimesteps) = true;
		}
	}

	return false;
}

void Usage()
{
	std::cout << "\nusage: ./run_solver path/to/problem.txt [path/to/solution.txt] \n"
	<< "If a path to a solution.txt is provided, the output will be written there.\n" << std::endl;
}

/**
 * @brief Main loop to parse command line args and run the solver.
 */
int main(int argc, char const *argv[])
{
	// Load in a problem.txt from CLI.
	if (argc < 2) {
		Usage(); // Give help message.
		throw std::runtime_error("Need to specify a filename for a problem.txt");
	}

	const std::string problem_path(argv[1]);

	// Set up the routing problem.
	RoutingProblem problem(problem_path);
	std::cout << "Loaded problem from: " << problem_path << std::endl;

	// Solve for shortest path.
	std::vector<Point2i> path;

	const auto start_t = std::chrono::high_resolution_clock::now();
	bool success = FindShortestPath(problem, &path);
	const auto finish_t = std::chrono::high_resolution_clock::now();
	const std::chrono::duration<double> elapsed = finish_t - start_t;

	if (success) {
		std::cout << "Found path!" << std::endl;
		const std::string solution = ConvertPathToString(path);
		std::cout << solution << std::endl;

		// If solution path is provided, write path to file.
		if (argc >= 3) {
			const std::string solution_path(argv[2]);
			std::cout << "Writing solution to: " << solution_path << std::endl;
			std::ofstream outfile(solution_path);
    	outfile << solution;
    	outfile.close();
		}
	} else {
		std::cout << "Could not find a path." << std::endl;
	}
	printf("runtime=%lf sec \n", elapsed.count());

	return 0;
}
