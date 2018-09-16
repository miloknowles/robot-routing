#include <iostream>
#include <priority_queue>

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
	return 0.0;
}

/**
 * @brief Trace back parent pointers to reconstruct a path from the goal node.
 */
std::vector<Point2i> ReconstructPath(const std::vector<Node>& nodes, const std::vector<int> parents,
																		 const int from_idx)
{
	// Expect from_idx to point to the goal node (should be the last).
	int current_idx = from_idx;

	std::vector<Point2i> path;

	while (current_idx > 0) {
		path.emplace_back(nodes.at(current_idx).point);
		current_idx = parents.at(current_idx);
	}

	return path;
}

bool FindShortestPath(const RoutingProblem& problem, std::vector<Node>* path)
{
	PriorityQueue pq;

	// Each time a node is expanded, it's stored here for path reconstruction later.
	std::vector<Node> explored_nodes;

	// Stores an index into explored_nodes to retrace paths.
	std::vector<int> parent_indices;

	// Add the start node.
	Node start_node(0, problem.OriginPoint());
	start_node.cost_so_far = 0;
	start_node.cost_using_node = HeuristicCostEstimate(problem, node);
	pq.push(start_node);

	while (!pq.empty()) {
		// Get the node with minimum cost_using_node.
		Node best = pq.top();
		explored_nodes.emplace_back(best);
		pq.pop();

		// If goal is found, retrace the path and terminate.
		if (best.point == problem.GoalPoint()) {
			*path = ReconstructPath(explored_nodes, parent_indices);
			return true;
		}

		// Add valid neighboring nodes.
		std::vector<Node> neighbors = problem.GetNeighbors(best);
		for (Node& neighbor : neighbors) {
			neighbor.cost_so_far = best.cost_so_far + 1; // All neighbors are 1 step away by definition.
			neighbor.cost_using_node = neighbor.cost_so_far + HeuristicCostEstimate(problem, node);
			neighbor.parent = &best
			pq.push(neighbor);
		}
	}

	return false;
}
