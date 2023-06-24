#include <gtest/gtest.h>

#include "MazeLib/Maze.h"

using namespace MazeLib;

TEST(MAZE_SIZE, MAZE_SIZE) {
  EXPECT_LE(MAZE_SIZE, MAZE_SIZE_MAX);
  EXPECT_EQ(MAZE_SIZE_MAX, std::pow(2, MAZE_SIZE_BIT));
}
