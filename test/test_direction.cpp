#include "Maze.h"
#include "gtest/gtest.h"

using namespace MazeLib;

TEST(Direction, RelativeDirection) {
  EXPECT_EQ(Direction(Direction::East - Direction::North), Direction::Right);
  EXPECT_EQ(Direction(Direction::South + Direction::Left), Direction::East);
  EXPECT_EQ(Direction(Direction::Right + Direction::Right), Direction::Back);
  EXPECT_EQ(Direction(-Direction::Left), Direction::Right);
}

TEST(Direction, isAlong_isDiag) {
  EXPECT_TRUE(Direction(Direction::East).isAlong());
  EXPECT_FALSE(Direction(Direction::NorthEast).isAlong());
  EXPECT_TRUE(Direction(Direction::SouthWest).isDiag());
  EXPECT_FALSE(Direction(Direction::West).isDiag());
}

TEST(Direction, Along4_Diag4) {
  EXPECT_EQ(Direction::Along4.size(), 4);
  for (const auto d : Direction::Along4) {
    EXPECT_TRUE(d.isAlong());
    EXPECT_FALSE(d.isDiag());
  }
  EXPECT_EQ(Direction::Diag4.size(), 4);
  for (const auto d : Direction::Diag4) {
    EXPECT_FALSE(d.isAlong());
    EXPECT_TRUE(d.isDiag());
  }
}

TEST(Direction, toChar) {
  EXPECT_EQ(Direction(Direction::East).toChar(), '>');
  EXPECT_EQ(Direction(Direction::East).toChar(), '>');
}
