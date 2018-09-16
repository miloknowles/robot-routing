#pragma once

#include <vector>
#include <iostream>

#include "utils.hpp"

namespace routing {

/**
 * @brief Represents a unique state on the infinite game board.
 */
struct Node {
	Node(const size_t ts, const Point2i pt) : timestep(ts), point(pt) {}
	size_t timestep;
	Point2i point;
	float cost_using_node = 0; // Estimated total cost using this node.
	float cost_so_far = 0; // Distance from the start to this node.
};

/**
 * @brief Abstraction for an instance of a routing problem.
 */
class RoutingProblem {
 public:
	RoutingProblem() = default;

	/**
	 * @brief Checks that a state (x, y, t) is valid (i.e does not collide with an obstacle or laser).
	 * The obstacle map stores a mod timestep in each cell, on which that cell is "occupied" with an
	 * obstacle. If the value is >= 5, then this cell is permanently occupied (barrier).
	 */
	bool IsNodeValid(const Node& node) const;

	/**
	 * @brief Gets the valid neighbors of a node by simulating forward 1 timestep.
	 */
	std::vector<Node> GetNeighbors(const Node& node) const;

	/**
	 * @brief Accessors.
	 */
	Point2i GoalPoint() { return goal_point; }
	Point2i OriginPoint() { return origin_point; }

 private:
 	const Point2i origin_point;
 	const Point2i goal_point;
 	const Grid<int> obstacle_map;
 	const Grid<Point2i*> wormhole_map;
};

}
