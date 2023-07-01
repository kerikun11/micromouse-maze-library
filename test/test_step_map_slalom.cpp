/**
 * @file test_step_map_slalom.cpp
 * @brief Unit Test for MazeLib::StepMapSlalom
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2023-07-01
 * @copyright Copyright 2023 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include <gtest/gtest.h>

#include "MazeLib/StepMapSlalom.h"

using namespace MazeLib;

TEST(StepMapSlalom, operator_left_shift_left_shift) {
  {
    std::stringstream ss;
    ss << StepMapSlalom::Index(1, 2, Direction::East);
    EXPECT_EQ(ss.str(), "(  1,  2, >)");
  }
  {
    std::stringstream ss;
    ss << StepMapSlalom::Index(1, 2, 0, Direction::NorthEast);
    EXPECT_EQ(ss.str(), "(  1,  2, >, ')");
  }
}
