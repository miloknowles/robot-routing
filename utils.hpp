#pragma once

#include <vector>
#include <iostream>

namespace routing {

/**
 * @brief Represents a 2D grid point by xy coordinates.
 */
struct Point2i
{
	int x; // Increasing right.
	int y; // Increasing up.
};

/**
 * @brief Manhattan distance between two integer points.
 */
float ManhattanDistance(const Point2i& p1, const Point2i& p2)
{
	return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
}

/**
 * @brief Represents a grid map as a 2D matrix.
 */
template <typename CellType>
class Grid {
	/**
	 * @brief Initialize an empty grid with dimensions.
	 */
	Grid(const size_t w, const size_t h, const CellType& init)
			: width_(w), height_(h)
	{
		// Initalize the grid with value "init" everywhere.
		grid_.resize(height_); // Should have height rows.
		for (size_t j = 0; j < height_; ++j) {
			grid_.at(j).resize(width_); // Should have width cols.
			for (size_t i = 0; i < width_; ++i) {
				grid_.at(j).at(i) = init;
			}
		}
	}

	const CellType& GetCell(const Point2i& xy) const
	{
		assert(xy.x < width_ && xy.y < height_);
		return grid_.at(xy.y).at(xy.x);
	}
	
	void SetCell(const Point2i& xy, const CellType& val)
	{
		assert(xy.x < width_ && xy.y < height_);
		grid_.at(xy.y).at(xy.x) = val;
	}

 private:
 	const size_t width_;
 	const size_t height_;
 	std::vector<std::vector<CellType>> grid_;
};

}
