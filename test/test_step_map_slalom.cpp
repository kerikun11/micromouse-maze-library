#include "MazeLib/StepMapSlalom.h"

#include <gtest/gtest.h>

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
