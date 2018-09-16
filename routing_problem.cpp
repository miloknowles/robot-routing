#include <ifstream>

#include "routing_problem.hpp"

namespace routing {

std::vector<std::string> ParseTuple(const std::string& tupstr)
{

	tupstr.erase(std::remove(tupstr.begin(), tupstr.end(), '('), tupstr.end());
	tupstr.erase(std::remove(tupstr.begin(), tupstr.end(), ')'), tupstr.end());
	tupstr.erase(std::remove(tupstr.begin(), tupstr.end(), ' '), tupstr.end());

	std::vector<int> tokens;
	std::stringstream ss(tupstr);
	std::string tmp;

	while (getline(ss, tmp, ',')) {
		tokens.emplace_back(tmp);
	}

	return tokens;
}

int main() 
{ 
      
    string line = "GeeksForGeeks is a must try"; 
      
    // Vector of string to save tokens 
    vector <string> tokens; 
      
    // stringstream class check1 
    stringstream check1(line); 
      
    string intermediate; 
      
    // Tokenizing w.r.t. space ' ' 
    while(getline(check1, intermediate, ' ')) 
    { 
        tokens.push_back(intermediate); 
    } 
      
    // Printing the token vector 
    for(int i = 0; i < tokens.size(); i++) 
        cout << tokens[i] << '\n'; 
} 

RoutingProblem::RoutingProblem(const std::string& filepath)
{
	const std::ifstream infile(filepath);
	const std::string orig_line, dest_line, barr_line, laser_line, worm_line;
	std::getline(infile, orig_line);
	std::getline(infile, dest_line);
	std::getline(infile, barr_line);
	std::getline(infile, laser_line);
	std::getline(infile, worm_line);
}

bool RoutingProblem::IsNodeValid(const Node& node)
{
	const int cell_active_timestep = obstacle_map_.GetCell(node.point);
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
			if (wormhole_map_.GetCell(point) != nullptr) {
				neighbors.at(ni).point = *wormhole_map_.GetCell(point);
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
