/**
 * @file test_maze_size.cpp
 * @brief Unit Test for MazeLib::SIZE
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2023-07-01
 * @copyright Copyright 2023 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include <gtest/gtest.h>

#include "MazeLib/Maze.h"

using namespace MazeLib;

TEST(MAZE_SIZE, MAZE_SIZE) {
  EXPECT_LE(MAZE_SIZE, MAZE_SIZE_MAX);
  EXPECT_EQ(MAZE_SIZE_MAX, std::pow(2, MAZE_SIZE_BIT));
}
