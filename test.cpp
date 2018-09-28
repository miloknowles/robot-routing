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
	const std::string problem_path("../sample/problem.txt");
	std::cout << "Loading from: " << problem_path << std::endl;
	RoutingProblem problem(problem_path);

	// Test the start and goal postions.
	EXPECT_EQ(2, problem.OriginPoint().x);
	EXPECT_EQ(3, problem.OriginPoint().y);
	EXPECT_EQ(8, problem.GoalPoint().x);
	EXPECT_EQ(1, problem.GoalPoint().y);

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
	EXPECT_EQ(10, wormhole_map->GetCell(Point2i(2, 2))->x);
	EXPECT_EQ(1, wormhole_map->GetCell(Point2i(2, 2))->y);
	// (10, 1) -> (2, 2)
	EXPECT_EQ(2, wormhole_map->GetCell(Point2i(10, 1))->x);
	EXPECT_EQ(2, wormhole_map->GetCell(Point2i(10, 1))->y);
}

// Run all the tests that were declared with TEST().
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
