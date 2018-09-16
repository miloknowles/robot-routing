#include "routing_problem.hpp"

namespace routing {

bool RoutingProblem::IsNodeValid(const Node& node)
{
	const int cell_active_timestep = obstacle_map.GetCell(node.point);
	return (cell_active_timestep < 5 && cell_active_timestep != node.timestep);
}

std::vector<Node> RoutingProblem::GetNeighbors(const Node& node)
{
	std::vector<Node> neighbors;
	neighbors.emplace_back(node.timestep + 1, Point2i(node.point.x + 1, node.point.y)); // Right.
	neighbors.emplace_back(node.timestep + 1, Point2i(node.point.x, node.point.y - 1)); // Down.
	neighbors.emplace_back(node.timestep + 1, Point2i(node.point.x - 1, node.point.y)); // Left.
	neighbors.emplace_back(node.timestep + 1, Point2i(node.point.x, node.point.y + 1)); // Up.

	// Apply any wormhole jumps every 3 seconds.
	if ((node.timestep + 1) % 3 == 0) {
		for (size_t ni = 0; ni < neighbors.size(); ++ni) {
			const Point2i& point = neighbors.at(ni).point;

			// If landed on a wormhole, instantaneously transport to partner coordinate.
			if (wormhole_map.GetCell(point) != nullptr) {
				neighbors.at(ni).point = *wormhole_map.GetCell(point);
			}
		}
	}

	// Check for collisions with barriers.
	std::vector<Node> neighbors_valid;
	for (const Node& neighbor : neighbors) {
		if (IsNodeValid(neighbor)) {
			neighbors_valid.emplace_back(neighbor);
		}
	}

	return neighbors_valid
}

}
