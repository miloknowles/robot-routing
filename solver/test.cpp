#include <gtest/gtest.h>

#include "routing_problem.hpp"
#include "utils.hpp"

using namespace routing;

static constexpr int kStaticObstacle = 5;

/**
 * @brief Make sure that grid methods behave properly.
 */
TEST(GridTest, TestCellValid)
{
  const int w = 10;
  const int h = 9;
  const Point2i min_cell_coord(2, -1);
  Grid<int> grid(w, h, min_cell_coord, 0);

  EXPECT_TRUE(grid.CellValid(Point2i(2, -1))); // Bottom left.
  EXPECT_TRUE(grid.CellValid(Point2i(11, 7))); // Top right.
  EXPECT_TRUE(grid.CellValid(Point2i(2, 7))); // Top left.
  EXPECT_TRUE(grid.CellValid(Point2i(11, -1))); // Bottom right;

  EXPECT_FALSE(grid.CellValid(Point2i(12, -1)));
  EXPECT_FALSE(grid.CellValid(Point2i(11, 8)));
  EXPECT_FALSE(grid.CellValid(Point2i(1, -1)));
  EXPECT_FALSE(grid.CellValid(Point2i(2, -2)));
}

/**
 * @brief Make sure that a problem is loaded in properly.
 */
TEST(RoutingProblemTest, TestConstructor)
{
  const std::string problem_path("../../sample/problem.txt");
  std::cout << "Loading from: " << problem_path << std::endl;
  RoutingProblem problem(problem_path);

  // Test the start and goal postions.
  EXPECT_EQ(Point2i(2, 3), problem.OriginPoint());
  EXPECT_EQ(Point2i(8, 1), problem.GoalPoint());

  // Test the location of static obstacles.
  const std::shared_ptr<Grid<int>> obstacle_grid = problem.ObstacleMap();
  EXPECT_EQ(kStaticObstacle, obstacle_grid->GetCell(Point2i(8, 2)));
  EXPECT_EQ(kStaticObstacle, obstacle_grid->GetCell(Point2i(7, 1)));
  EXPECT_EQ(kStaticObstacle, obstacle_grid->GetCell(Point2i(4, 5)));
  EXPECT_EQ(kStaticObstacle, obstacle_grid->GetCell(Point2i(9, 5)));

  // Test the temporal location of lasers.
  for (int yi = 1; yi < 3; ++yi) { // North beam.
    const Point2i test_pt(9, 5 + yi);

    EXPECT_EQ(0, obstacle_grid->GetCell(test_pt));
  }
  for (int xi = 1; xi < 3; ++xi) { // East beam.
    const Point2i test_pt(9 + xi, 5);
    EXPECT_EQ(1, obstacle_grid->GetCell(test_pt));
  }
  for (int yi = 1; yi < 7; ++yi) { // South beam.
    const Point2i test_pt(9, 5 - yi);
    EXPECT_EQ(2, obstacle_grid->GetCell(test_pt));
  }
  for (int xi = 1; xi < 5; ++xi) { // West beam.
    const Point2i test_pt(9 - xi, 5);
    EXPECT_EQ(3, obstacle_grid->GetCell(test_pt));
  }

  // Test wormhole locations.
  std::shared_ptr<Grid<Point2i*>> wormhole_map = problem.WormholeMap();

  // (2, 2) -> (10, 1)
  EXPECT_EQ(Point2i(10, 1), *wormhole_map->GetCell(Point2i(2, 2)));
  // (10, 1) -> (2, 2)
  EXPECT_EQ(Point2i(2, 2), *wormhole_map->GetCell(Point2i(10, 1)));
}

/**
 * @brief Make sure that neighbors of a node are correct.
 */
TEST(SolverTest, TestGetNeighbors)
{
  const std::string problem_path("../../sample/problem.txt");
  std::cout << "Loading from: " << problem_path << std::endl;
  RoutingProblem problem(problem_path);

  Node p1(0, Point2i(2, 2));
  const std::vector<Node> n1 = problem.GetNeighbors(p1);
  EXPECT_EQ(4, n1.size());
  for (const Node& n : n1) {
    EXPECT_EQ(1, n.timestep);
  }

  Node p2(2, Point2i(2, 1));
  const std::vector<Node> n2 = problem.GetNeighbors(p2);
  EXPECT_EQ(4, n2.size());
  for (const Node& n : n2) {
    std::cout << n.point.x << " " << n.point.y << std::endl;
    EXPECT_EQ(3, n.timestep);
  }
}

/**
 * @brief Test setup for problem1 to debug.
 */
TEST(SolverTest, TestProblem1)
{
  const std::string problem_path("../../problem1/problem.txt");
  std::cout << "Loading from: " << problem_path << std::endl;
  RoutingProblem problem(problem_path);

  // Check start and goal locations.
  EXPECT_EQ(Point2i(2, 3), problem.OriginPoint());
  EXPECT_EQ(Point2i(8, 1), problem.GoalPoint());

  // Check obstacle locations.
  std::shared_ptr<Grid<int>> obstacle_grid = problem.ObstacleMap();
  EXPECT_EQ(kStaticObstacle, obstacle_grid->GetCell(Point2i(7, 1)));
  EXPECT_EQ(kStaticObstacle, obstacle_grid->GetCell(Point2i(8, 0)));
  EXPECT_EQ(kStaticObstacle, obstacle_grid->GetCell(Point2i(8, 2)));
  EXPECT_EQ(kStaticObstacle, obstacle_grid->GetCell(Point2i(9, 2)));
  EXPECT_EQ(kStaticObstacle, obstacle_grid->GetCell(Point2i(10, 2)));

  // Make sure the path to the goal is clear.
  EXPECT_EQ(-1, obstacle_grid->GetCell(Point2i(2, 3)));
  EXPECT_EQ(-1, obstacle_grid->GetCell(Point2i(8, 1)));
  EXPECT_EQ(-1, obstacle_grid->GetCell(Point2i(9, 1)));
}

// Run all the tests that were declared with TEST().
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
