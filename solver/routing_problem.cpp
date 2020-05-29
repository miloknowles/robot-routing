#include <fstream>
#include <algorithm>

#include "routing_problem.hpp"

namespace routing {

// An obstacle in the grid with this value is active on every timestep (static).
static constexpr int kStaticObstacle = 5;

RoutingProblem::RoutingProblem(const std::string& filepath)
{
  // Open the problem.txt and retrieve the 5 lines.
  std::ifstream infile(filepath);
  std::string orig_line, dest_line, barr_line, laser_line, worm_line;
  std::getline(infile, orig_line);
  std::getline(infile, dest_line);
  std::getline(infile, barr_line);
  std::getline(infile, laser_line);
  std::getline(infile, worm_line);

  // Parse all lines into character lists.
  const std::vector<std::string> orig = ParseTuple(orig_line);
  const std::vector<std::string> dest = ParseTuple(dest_line);
  const std::vector<std::string> barriers = ParseTuple(barr_line);
  const std::vector<std::string> lasers = ParseTuple(laser_line);
  const std::vector<std::string> wormholes = ParseTuple(worm_line);

  // Sanity checks on length.
  assert(orig.size() == 2 && dest.size() == 2);
  assert(barriers.size() % 2 == 0); // Should contain pairs of coordinates.
  assert(lasers.size() % 3 == 0); // Should contain triples.
  assert(wormholes.size() % 4 == 0); // Should contain two pairs.

  // Parse the origin point and goal point.
  origin_point_ = Point2i(std::stoi(orig[0]), std::stoi(orig[1]));
  goal_point_ = Point2i(std::stoi(dest[0]), std::stoi(dest[1]));

  // Find the bounding coordinates of the board.
  Point2i min_corner(std::min(origin_point_.x, goal_point_.x), std::min(origin_point_.y, goal_point_.y));
  Point2i max_corner(std::max(origin_point_.x, goal_point_.x), std::max(origin_point_.y, goal_point_.y));

  // Get the location of all barriers.
  std::vector<Point2i> barrier_points;
  for (size_t i = 0; i < barriers.size() / 2; ++i) {
    const Point2i xy(std::stoi(barriers.at(2*i)), std::stoi(barriers.at(2*i + 1)));
    UpdateMinMaxPoint(xy, &min_corner, &max_corner);
    barrier_points.emplace_back(xy);
  }

  // Get the location of all lasers.
  for (size_t i = 0; i < lasers.size() / 3; ++i) {
    const Point2i xy(std::stoi(lasers.at(3*i)), std::stoi(lasers.at(3*i+1)));
    const std::string dir = lasers.at(3*i + 2);
    lasers_.emplace_back(xy, CardinalDirectionToVector(dir));
    UpdateMinMaxPoint(xy, &min_corner, &max_corner);
  }

  // Finally, get the location of all wormholes.
  for (size_t i = 0; i < wormholes.size() / 4; ++i) {
    const Point2i p1(std::stoi(wormholes.at(4*i)), std::stoi(wormholes.at(4*i+1)));
    const Point2i p2(std::stoi(wormholes.at(4*i+2)), std::stoi(wormholes.at(4*i+3)));
    UpdateMinMaxPoint(p1, &min_corner, &max_corner);
    UpdateMinMaxPoint(p2, &min_corner, &max_corner);
    wormholes_.emplace_back(p1, p2);
  }

  // Now we know the boundaries of the grid - 2 squares of padding around the obstacle bounding box.
  // See APPENDIX.md for my argument why.
  const int width = max_corner.x - min_corner.x + 4 + 1;
  const int height = max_corner.y - min_corner.y + 4 + 1;
  const Point2i grid_min_corner = Point2i(min_corner.x - 2, min_corner.y - 2);
  obstacle_map_ = Grid<int>::Create(width, height, grid_min_corner, -1);

  // Set static obstacle locations in grid.
  for (const Point2i& pt : barrier_points) {
    obstacle_map_->SetCell(pt, kStaticObstacle);
  }
  for (const Laser& laser : lasers_) {
    obstacle_map_->SetCell(laser.first, kStaticObstacle);
  }

  // Now set the location of temporal obstacles (laser beams).
  for (const Laser& laser : lasers_) {
    Point2i direction = laser.second; // Direction of the laser at the current timestep.

    for (int step = 0; step < 4; ++step) {
      // Start on the laser base, and cast ray outwards.
      Point2i ray(laser.first.x + direction.x, laser.first.y + direction.y);
      while (obstacle_map_->CellValid(ray) && (obstacle_map_->GetCell(ray) != kStaticObstacle)) {
        obstacle_map_->SetCell(ray, step); // The step indicates when this cell has the laser in it.

        ray.x += direction.x;
        ray.y += direction.y;
      }
      // Rotate 90 deg clockwise.
      direction = Point2i(direction.y, -1*direction.x);
    }
  }
  // Finally, fill in the wormhole map. Make each wormhole location point to the other.
  if (wormholes_.size() > 0) {
    wormhole_map_ = Grid<Point2i*>::Create(width, height, grid_min_corner, nullptr);
    for (Wormhole& wh : wormholes_) {
      wormhole_map_->SetCell(wh.first, &wh.second);
      wormhole_map_->SetCell(wh.second, &wh.first);
    }
  }
}

bool RoutingProblem::IsNodeValid(const Node& node) const
{
  // Make sure the node has a point that is within bounds.
  if (!obstacle_map_->CellValid(node.point)) {
    return false;
  }
  // Make sure there are no collisions at this timestep.
  const int cell_active_timestep = obstacle_map_->GetCell(node.point);
  return (cell_active_timestep != kStaticObstacle &&
          cell_active_timestep != (node.timestep % 4));
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
      if (wormhole_map_ != nullptr && wormhole_map_->CellValid(point) &&
          wormhole_map_->GetCell(point) != nullptr) {
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
