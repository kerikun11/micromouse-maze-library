#include "Maze.h"
#include "gtest/gtest.h"

using namespace MazeLib;

TEST(MazeTest, Position) {
  const auto p = Position(0, 0);
  EXPECT_EQ(p, p);
}
