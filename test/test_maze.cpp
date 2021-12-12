#include "MazeLib/Maze.h"
#include <gtest/gtest.h>
#include <algorithm>

using namespace MazeLib;

TEST(Maze, parse_from_file) {
  /* ファイルから迷路情報を取得 */
  Maze maze;
  const std::string file_path = "../mazedata/data/32MM2019HX.maze";
  EXPECT_TRUE(maze.parse(file_path.c_str()));
  EXPECT_TRUE(maze.canGo(Position(0, 0), Direction::North));
  for (const auto clear : {true, false})
    EXPECT_TRUE(maze.backupWallRecordsToFile("data.bin", clear));
  EXPECT_TRUE(maze.restoreWallRecordsFromFile("data.bin"));
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
  maze.print({Position(1, 1)});

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

TEST(Maze, parse_from_string_array) {
  const std::vector<std::string> mazeData = {
      "a6666663ba627a63", "c666663c01a43c39", "a2623b879847c399",
      "9c25c05b85e23999", "9a43a5b85e219999", "9c385b85e25d9999",
      "9e05b85e25a39999", "9a5b85ba1a599999", "99b85b84587c5999",
      "9c05b85a20666599", "c3db85a5d9bbbb99", "b87847c639800059",
      "85e466665c5dddb9", "8666666666666645", "c666666666666663",
      "e666666666666665",
  };
  /* parameter */
  const auto maze_data = mazeData;
  const int maze_size = mazeData.size();
  const std::string output_filename = "output.maze";
  const Positions goals = {
      Position(7, 7),
      Position(8, 7),
      Position(7, 8),
      Position(8, 8),
  };
  /* process */
  Maze sample;
  std::ofstream of(output_filename);
  sample.parse(maze_data, maze_size);
  sample.setGoals(goals);
  sample.print(of, maze_size);
  sample.print(std::cout, maze_size);
}
