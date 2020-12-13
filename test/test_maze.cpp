#include "Maze.h"
#include "gtest/gtest.h"

using namespace MazeLib;

TEST(Maze, parse_from_file) {
  /* ファイルから迷路情報を取得 */
  Maze maze;
  const std::string file_path = "../mazedata/data/32MM2019HX.maze";
  EXPECT_TRUE(maze.parse(file_path.c_str()));
  EXPECT_TRUE(maze.canGo(Position(0, 0), Direction::North));
}

TEST(Maze, parse_from_istream) {
  std::stringstream maze_stream;
  maze_stream << R"(
+---+---+---+---+---+---+---+---+---+
|               |                   |
+   +---+   +   +   +---+---+---+   +
|       |   |   |   |               |
+---+   +   +   +   +   +---+---+---+
|       |   |       |               |
+   +---+   +---+---+---+---+---+   +
|       |   | G   G   G |           |
+---+   +   +   +   +   +   +---+---+
|       |   | G   G   G |           |
+   +---+   +   +   +   +---+---+   +
|       |   | G   G   G |       |   |
+---+   +   +   +---+---+   +   +   +
|       |   |   |       |   |   |   |
+   +---+   +   +   +   +   +   +   +
|       |   |   |   |   |   |   |   |
+   +   +   +   +   +   +   +   +   +
|   | S |   |       |       |       |
+---+---+---+---+---+---+---+---+---+
)";
  Maze maze;
  maze_stream >> maze;
  // maze.parse(maze_stream);

  EXPECT_EQ(maze.getStart(), Position(1, 0));

  Positions expected_goals;
  for (auto x : {3, 4, 5})
    for (auto y : {3, 4, 5})
      expected_goals.push_back(Position(x, y));

  EXPECT_EQ(expected_goals.size(), maze.getGoals().size());
  for (const auto g : maze.getGoals())
    EXPECT_TRUE(std::find(expected_goals.cbegin(), expected_goals.cend(), g) !=
                expected_goals.cend());
}
