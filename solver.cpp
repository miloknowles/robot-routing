#include <iostream>
#include <queue>
#include <algorithm>
#include <string>
#include <tuple>
#include <unordered_set>

#include "routing_problem.hpp"
#include "utils.hpp"

using namespace routing;

typedef std::tuple<size_t, int, int> SearchPoint;

/**
 * @brief Custom comparator for nodes, used in priority queue.
 * Note: greater than is used to make this a min queue.
 */
class CompareNode {
 public:
  bool operator()(const Node& lhs, const Node& rhs)
  {
  	return (lhs.timestep > rhs.timestep);
  }
};

typedef std::priority_queue<Node, std::vector<Node>, CompareNode> PriorityQueue;

/**
 * @brief Returns the estimated cost to go from a node.
 * Note: this must be admissible in order for A* optimality.
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
 * @brief Trace back parent pointers to reconstruct a path from the goal node.
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

bool FindShortestPath(const RoutingProblem& problem, std::vector<Point2i>* path)
{
	// Maintain a grid of 3D points that have been expanded to avoid revisiting.
	const std::pair<size_t, size_t>& dims = problem.ObstacleMap()->Dimensions();
	const Point2i& min_cell = problem.ObstacleMap()->MinCellCoord();
	Grid<std::vector<bool>> expanded(dims.first, dims.second, min_cell, std::vector<bool>(12, 0));

	// Best-first search will use a priority queue.
	PriorityQueue pq;

	// Each time a node is expanded, it's stored here for path reconstruction later.
	std::vector<Node> explored_nodes;

	// Add the start node: parent = -1 by default.
	Node start_node(0, problem.OriginPoint());
	start_node.cost_using_node = HeuristicCostEstimate(problem, start_node);
	pq.push(start_node);

	int frontier = 0;

	while (!pq.empty()) {
		// Expand the node with minimum cost_using_node.
		const Node best = pq.top();
		explored_nodes.emplace_back(best);
		pq.pop();

		if (best.timestep > frontier) {
			frontier = best.timestep;
			std::cout << "Frontier: " << frontier << std::endl;
			std::cout << "queue size: " << pq.size() << std::endl;
		}

		// If goal is found, retrace the path and return success.
		if (best.point == problem.GoalPoint()) {
			*path = ReconstructPath(explored_nodes, explored_nodes.size()-1);
			return true;
		}

		// Add valid neighboring nodes.
		std::vector<Node> neighbors = problem.GetNeighbors(best);

		for (Node& neighbor : neighbors) {
			// Skip this neighbor if an equivalent point in the search space has already been explored.
			if (expanded.GetCell(neighbor.point).at(neighbor.timestep % 12) == true) {
				continue;
			}

			neighbor.cost_so_far = best.cost_so_far + 1; // All neighbors are 1 step away by definition.
			neighbor.cost_using_node = neighbor.cost_so_far + HeuristicCostEstimate(problem, neighbor);
			neighbor.parent = explored_nodes.size() - 1; // Point to the current expanding node.
			pq.push(neighbor);

			// Mark that this search point (x, y, t % 12) has been added to avoid duplicates.
			std::vector<bool>* const expanded_point = expanded.GetCellMutable(neighbor.point);
			expanded_point->at(neighbor.timestep % 12) = true;
		}
	}

	return false;
}

/**
 * @brief Main loop to parse command line args and run the solver.
 */
int main(int argc, char const *argv[])
{
	// Load in a problem.txt from CLI.
	if (argc < 2) {
		throw std::runtime_error("Need to specify a filename for a problem.txt");
	}

	const std::string problem_path(argv[1]);

	// Set up the routing problem.
	RoutingProblem problem(problem_path);
	std::cout << "Loaded problem from: " << problem_path << std::endl;

	// Solve for shortest path.
	std::vector<Point2i> path;
	bool success = FindShortestPath(problem, &path);

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

	return 0;
}
