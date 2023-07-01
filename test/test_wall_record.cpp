/**
 * @file test_wall_record.cpp
 * @brief Unit Test for MazeLib::WallRecord
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2023-07-01
 * @copyright Copyright 2023 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include <gtest/gtest.h>

#include "MazeLib/Maze.h"

using namespace MazeLib;

TEST(WallRecord, operator_left_shift_left_shift) {
  std::stringstream ss;
  ss << WallRecord(1, 2, Direction::East, true);
  EXPECT_EQ(ss.str(), "(  1,  2, >, true)");
}
