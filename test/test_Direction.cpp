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

TEST(Direction, getAlong4_getDiag4) {
  for (const auto d : Direction::getAlong4())
    EXPECT_TRUE(d.isAlong());
  for (const auto d : Direction::getDiag4())
    EXPECT_TRUE(d.isDiag());
}
