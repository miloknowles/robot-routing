#pragma once

#include <vector>
#include <iostream>
#include <assert.h>
#include <memory>

namespace routing {

/**
 * @brief Represents a 2D grid point by xy coordinates.
 */
struct Point2i
{
	Point2i() = default;
	Point2i(const int x, const int y) : x(x), y(y) {}
	int x = 0; // Increasing right.
	int y = 0; // Increasing up.
};

/**
 * @brief Manhattan distance between two integer points.
 */
inline float ManhattanDistance(const Point2i& p1, const Point2i& p2)
{
	return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
}

/**
 * @brief Represents a grid map as a 2D matrix.
 */
template <typename CellType>
class Grid {
 public:
	/**
	 * @brief Initialize an empty grid with dimensions.
	 */
	Grid(const size_t w, const size_t h, const Point2i& min_cell, const CellType& init)
			: width_(w), height_(h), min_cell_coord_(min_cell)
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
	/**
	 * @brief Convenience method to create a shared pointer to a grid.
	 */
	static std::shared_ptr<Grid<CellType>> Create(
		const size_t w, const size_t h, const Point2i& min_cell, const CellType& init)
	{
		return std::make_shared<Grid<CellType>>(w, h, min_cell, init);
	}

	const CellType& GetCell(const Point2i& xy) const
	{
		assert(xy.x < width_ && xy.y < height_);
		return grid_.at(xy.y - min_cell_coord_.y).at(xy.x - min_cell_coord_.x);
	}
	
	void SetCell(const Point2i& xy, const CellType& val)
	{
		assert(xy.x < width_ && xy.y < height_);
		grid_.at(xy.y - min_cell_coord_.y).at(xy.x - min_cell_coord_.x) = val;
	}

	bool CellValid(const Point2i& xy) const
	{
		const int x = (xy.x - min_cell_coord_.x);
		const int y = (xy.y - min_cell_coord_.y);
		return x >= 0 && x < width_ && y >= 0 && y < height_;
	}

 private:
 	const size_t width_;
 	const size_t height_;
 	const Point2i min_cell_coord_;
 	std::vector<std::vector<CellType>> grid_;
};

}
