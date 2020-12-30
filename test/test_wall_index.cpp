#include "Maze.h"
#include "gtest/gtest.h"

using namespace MazeLib;

TEST(WallIndex, isInsideOfField) {
  EXPECT_TRUE(WallIndex({0, 0}, Direction::East).isInsideOfField());
  EXPECT_TRUE(WallIndex({0, 0}, Direction::North).isInsideOfField());
  EXPECT_TRUE(WallIndex({MAZE_SIZE - 1, MAZE_SIZE - 1}, Direction::West)
                  .isInsideOfField());
  EXPECT_TRUE(WallIndex({MAZE_SIZE - 1, MAZE_SIZE - 1}, Direction::South)
                  .isInsideOfField());
  EXPECT_FALSE(WallIndex(Position(0, 0), Direction::West).isInsideOfField());
  EXPECT_FALSE(WallIndex(Position(0, 0), Direction::South).isInsideOfField());
  EXPECT_FALSE(
      WallIndex({MAZE_SIZE - 1, 0}, Direction::East).isInsideOfField());
  EXPECT_FALSE(
      WallIndex({0, MAZE_SIZE - 1}, Direction::North).isInsideOfField());
}

TEST(WallIndex, operator_left_shift_left_shift) {
  std::stringstream ss;
  ss << WallIndex(1, 2, 0);
  EXPECT_EQ(ss.str(), "(  1,  2, >)");
}
