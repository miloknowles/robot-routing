#include <fstream>
#include <algorithm>
#include <sstream>
#include <string>

#include "routing_problem.hpp"

namespace routing {

/**
 * @brief Parses a tuple of the form (a, b, c, ...). Assumes values are comma-separated.
 */
std::vector<std::string> ParseTuple(const std::string& input_str)
{
	// Remove spaces and parentheses.
	std::string tupstr = input_str;
	tupstr.erase(std::remove(tupstr.begin(), tupstr.end(), '('), tupstr.end());
	tupstr.erase(std::remove(tupstr.begin(), tupstr.end(), ')'), tupstr.end());
	tupstr.erase(std::remove(tupstr.begin(), tupstr.end(), ' '), tupstr.end());

	std::vector<std::string> tokens;
	std::stringstream ss(tupstr);
	std::string tmp;

	while (std::getline(ss, tmp, ',')) {
		tokens.emplace_back(tmp);
	}

	return tokens;
}

RoutingProblem::RoutingProblem(const std::string& filepath)
{
	std::ifstream infile(filepath);
	std::string orig_line, dest_line, barr_line, laser_line, worm_line;
	std::getline(infile, orig_line);
	std::getline(infile, dest_line);
	std::getline(infile, barr_line);
	std::getline(infile, laser_line);
	std::getline(infile, worm_line);

	// Parse the origin and destination tuples.
	const std::vector<std::string> orig = ParseTuple(orig_line);
	const std::vector<std::string> dest = ParseTuple(dest_line);
	assert(orig.size() == 2 && dest.size() == 2);
	origin_point_ = Point2i(std::stoi(orig[0]), std::stoi(orig[1]));
	goal_point_ = Point2i(std::stoi(dest[0]), std::stoi(dest[1]));
}

bool RoutingProblem::IsNodeValid(const Node& node) const
{
	const int cell_active_timestep = obstacle_map_->GetCell(node.point);
	return (cell_active_timestep < 5 && cell_active_timestep != node.timestep);
}

std::vector<Node> RoutingProblem::GetNeighbors(const Node& node) const
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
			if (wormhole_map_->GetCell(point) != nullptr) {
				neighbors.at(ni).point = *wormhole_map_->GetCell(point);
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

	return neighbors_valid;
}

}
