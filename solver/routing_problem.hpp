#pragma once

#include <vector>
#include <iostream>
#include <string>
#include <memory>

#include "utils.hpp"

namespace routing {

typedef std::pair<Point2i, Point2i> Wormhole;
typedef std::pair<Point2i, Point2i> Laser;

/**
 * @brief Represents a unique state on the infinite game board.
 */
struct Node {
  Node() = delete;
  Node(const size_t ts, const Point2i pt) : timestep(ts), point(pt) {}

  size_t timestep; // The cost-so-far from the start node.
  Point2i point;
  float cost_using_node = 0; // Estimated total cost using this node.
  int parent = -1;
};

inline void PrintNode(const Node& best)
{
  printf("t=%zu p=(%d, %d) cost_using=%f",
         best.timestep, best.point.x, best.point.y, best.cost_using_node);
}

/**
 * @brief Abstraction for an instance of a routing problem.
 */
class RoutingProblem {
 public:
  RoutingProblem(const std::string& filepath);

  /**
   * @brief Checks that a state (x, y, t) is valid (i.e does not collide with an obstacle or laser).
   * The obstacle map stores a mod timestep in each cell, on which that cell is "occupied" with an
   * obstacle. If the value is >= 5, then this cell is permanently occupied (barrier).
   */
  bool IsNodeValid(const Node& node) const;

  /**
   * @brief Gets the valid neighbors of a node by simulating forward 1 timestep.
   * Note: all nodes will have their time step equal to node.timestep + 1.
   */
  std::vector<Node> GetNeighbors(const Node& node) const;

  /**
   * @brief Accessors.
   */
  const Point2i& GoalPoint() const { return goal_point_; }
  const Point2i& OriginPoint() const { return origin_point_; }
  const std::vector<Wormhole>& Wormholes() const { return wormholes_; }
  const std::shared_ptr<Grid<int>>& ObstacleMap() const { return obstacle_map_; }
   const std::shared_ptr<Grid<Point2i*>>& WormholeMap() const { return wormhole_map_; }

 private:
   Point2i origin_point_;
   Point2i goal_point_;
   std::shared_ptr<Grid<int>> obstacle_map_;
   std::shared_ptr<Grid<Point2i*>> wormhole_map_;
   std::vector<Laser> lasers_;
  std::vector<Wormhole> wormholes_;
};

}
