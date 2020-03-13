#include "Maze.h"
#include "gtest/gtest.h"

using namespace MazeLib;

TEST(Maze, parse) {
  /* ファイルから迷路情報を取得 */
  Maze maze;
  const std::string file_path = "../mazedata/32MM2019HX.maze";
  EXPECT_TRUE(maze.parse(file_path.c_str()));
  EXPECT_TRUE(maze.canGo(Position(0, 0), Direction::North));
}
