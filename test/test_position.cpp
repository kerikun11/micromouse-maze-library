#include "Maze.h"
#include "gtest/gtest.h"

using namespace MazeLib;

TEST(Position, next) {
  EXPECT_EQ(Position(0, 0).next(Direction::East), Position(1, 0));
  EXPECT_EQ(Position(0, 0).next(Direction::North), Position(0, 1));
  EXPECT_EQ(Position(0, 0).next(Direction::West), Position(-1, 0));
  EXPECT_EQ(Position(0, 0).next(Direction::South), Position(0, -1));

  EXPECT_EQ(Position(0, 0).next(Direction::NorthEast), Position(1, 1));
  EXPECT_EQ(Position(0, 0).next(Direction::NorthWest), Position(-1, 1));
  EXPECT_EQ(Position(0, 0).next(Direction::SouthEast), Position(1, -1));
  EXPECT_EQ(Position(0, 0).next(Direction::SouthWest), Position(-1, -1));
}

TEST(Position, isInsideOfField) {
  EXPECT_TRUE(Position(0, 0).isInsideOfField());
  EXPECT_TRUE(Position(MAZE_SIZE - 1, MAZE_SIZE - 1).isInsideOfField());
  EXPECT_FALSE(Position(-1, 0).isInsideOfField());
  EXPECT_FALSE(Position(0, -1).isInsideOfField());
  EXPECT_FALSE(Position(0, MAZE_SIZE).isInsideOfField());
  EXPECT_FALSE(Position(MAZE_SIZE, 0).isInsideOfField());
}

TEST(Position, rotate) {
  EXPECT_EQ(Position(2, 0).rotate(Direction::South), Position(0, -2));
  EXPECT_EQ(Position(2, 3).rotate(Direction::North, Position(2, 1)),
            Position(0, 1));
}

TEST(Position, operator_left_shift_left_shift) {
  std::stringstream ss;
  ss << Position(1, 2);
  EXPECT_EQ(ss.str(), "(  1,  2)");
}
