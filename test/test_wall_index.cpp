/**
 * @file test_wall_index.cpp
 * @brief Unit Test for MazeLib::WallIndex
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2023-07-01
 * @copyright Copyright 2023 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include <gtest/gtest.h>

#include "MazeLib/Maze.h"

using namespace MazeLib;

TEST(WallIndex, next) {
  WallIndex e = WallIndex(Position(0, 0), Direction::East);
  EXPECT_EQ(e.next(Direction::East),
            WallIndex(Position(1, 0), Direction::East));
  EXPECT_EQ(e.next(Direction::NorthEast),
            WallIndex(Position(1, 0), Direction::North));
  EXPECT_EQ(e.next(Direction::North),
            WallIndex(Position(0, 1), Direction::East));
  EXPECT_EQ(e.next(Direction::NorthWest),
            WallIndex(Position(0, 0), Direction::North));
  EXPECT_EQ(e.next(Direction::West),
            WallIndex(Position(0, 0), Direction::West));
  EXPECT_EQ(e.next(Direction::SouthWest),
            WallIndex(Position(0, 0), Direction::South));
  EXPECT_EQ(e.next(Direction::South),
            WallIndex(Position(0, -1), Direction::East));
  EXPECT_EQ(e.next(Direction::SouthEast),
            WallIndex(Position(1, 0), Direction::South));
  WallIndex n = WallIndex(Position(0, 0), Direction::North);
  EXPECT_EQ(n.next(Direction::East),
            WallIndex(Position(1, 0), Direction::North));
  EXPECT_EQ(n.next(Direction::NorthEast),
            WallIndex(Position(0, 1), Direction::East));
  EXPECT_EQ(n.next(Direction::North),
            WallIndex(Position(0, 1), Direction::North));
  EXPECT_EQ(n.next(Direction::NorthWest),
            WallIndex(Position(0, 1), Direction::West));
  EXPECT_EQ(n.next(Direction::West),
            WallIndex(Position(-1, 0), Direction::North));
  EXPECT_EQ(n.next(Direction::SouthWest),
            WallIndex(Position(0, 0), Direction::West));
  EXPECT_EQ(n.next(Direction::South),
            WallIndex(Position(0, 0), Direction::South));
  EXPECT_EQ(n.next(Direction::SouthEast),
            WallIndex(Position(0, 0), Direction::East));
}

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
