#include <gtest/gtest.h>

#include "MazeLib/Maze.h"

using namespace MazeLib;

TEST(WallRecord, operator_left_shift_left_shift) {
  std::stringstream ss;
  ss << WallRecord(1, 2, Direction::East, true);
  EXPECT_EQ(ss.str(), "(  1,  2, >, true)");
}
