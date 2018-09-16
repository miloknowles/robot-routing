#include <iostream>
#include <priority_queue>
#include <algorithm>

#include "routing_problem.hpp"
#include "utils.hpp"

using namespace routing;

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

	while (current_idx > 0) {
		path.emplace_back(nodes.at(current_idx).point);
		current_idx = nodes.at(current_idx).parent;
	}

	return path;
}

bool FindShortestPath(const RoutingProblem& problem, std::vector<Node>* path)
{
	PriorityQueue pq;

	// Each time a node is expanded, it's stored here for path reconstruction later.
	std::vector<Node> explored_nodes;

	// Add the start node - parent = -1 by default.
	Node start_node(0, problem.OriginPoint());
	start_node.cost_using_node = HeuristicCostEstimate(problem, node);
	pq.push(start_node);

	while (!pq.empty()) {
		// Expand the node with minimum cost_using_node.
		Node best = pq.top();
		explored_nodes.emplace_back(best);
		pq.pop();

		// If goal is found, retrace the path and return success.
		if (best.point == problem.GoalPoint()) {
			*path = ReconstructPath(explored_nodes, explored_nodes.size() - 1);
			return true;
		}

		// Add valid neighboring nodes.
		std::vector<Node> neighbors = problem.GetNeighbors(best);
		for (Node& neighbor : neighbors) {
			neighbor.cost_so_far = best.cost_so_far + 1; // All neighbors are 1 step away by definition.
			neighbor.cost_using_node = neighbor.cost_so_far + HeuristicCostEstimate(problem, node);
			neighbor.parent = explored_nodes.size() - 1; // Point to the current expanding node.
			pq.push(neighbor);
		}
	}

	return false;
}
