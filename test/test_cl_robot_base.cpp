#include "MazeLib/CLRobotBase.h"

#include <gtest/gtest.h>

using namespace MazeLib;

TEST(CLRobotBase, CLRobotBase) {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/data/";
  // const std::string filename = "32MM2019HX.maze";
  const std::string filename = "16MM2020CX.maze";
  // const std::string filename = "09MM2019C_Cheese_cand.maze";
  Maze maze_target;
  EXPECT_TRUE(maze_target.parse((mazedata_dir + filename).c_str()));
  CLRobotBase robot(maze_target);
  robot.replaceGoals(maze_target.getGoals());

  /* Search Run */
  robot.resetLastWalls();
  EXPECT_TRUE(robot.isSolvable());
  EXPECT_FALSE(robot.isComplete());
  EXPECT_TRUE(robot.searchRun());
  robot.printInfo();
  robot.printSearchResult();
  for (const auto diag_enabled : {false, true}) {
    EXPECT_TRUE(robot.calcShortestDirections(diag_enabled));
    EXPECT_TRUE(robot.getSearchAlgorithm().getShortestCost());
    EXPECT_TRUE(robot.fastRun(diag_enabled));
    robot.printPath();
  }

  /* Other Run */
  EXPECT_TRUE(robot.searchRun());
  EXPECT_TRUE(robot.positionIdentifyRun());

  /* StepMap */
  for (const auto simple : {true, false}) {
    const bool known_only = 0;
    const Maze& maze = maze_target;
    StepMap map;
    auto shortest_dirs = map.calcShortestDirections(
        maze, maze.getStart(), maze.getGoals(), known_only, simple);
    EXPECT_FALSE(shortest_dirs.empty());
    map.appendStraightDirections(maze, shortest_dirs, known_only, false);
    map.print(maze, shortest_dirs);
    map.print(maze);
    map.printFull(maze, shortest_dirs);
    map.printFull(maze);
    maze.print(shortest_dirs);
  }

  /* StepMapWall */
  for (const auto simple : {true, false}) {
    const bool known_only = 0;
    const Maze& maze = maze_target;
    StepMapWall map;
    auto shortest_dirs = map.calcShortestDirections(maze, known_only, simple);
    EXPECT_FALSE(shortest_dirs.empty());
    map.appendStraightDirections(maze, shortest_dirs);
    map.print(maze, shortest_dirs);
    map.print(maze);
    maze.print(StepMapWall::convertWallIndexDirectionsToPositionDirections(
                   shortest_dirs, WallIndex(Position(0, 0), Direction::North)),
               maze.getStart());
  }

  /* StepMapSlalom */
  for (const auto diag_enabled : {false, true}) {
    const bool known_only = 0;
    const Maze& maze = maze_target;
    StepMapSlalom map;
    StepMapSlalom::Indexes shortest_indexes;
    map.update(maze, StepMapSlalom::EdgeCost(),
               StepMapSlalom::convertDestinations(maze.getGoals()), known_only,
               diag_enabled);
    EXPECT_TRUE(map.genPathFromMap(shortest_indexes));
    map.print(maze, shortest_indexes);
    auto shortest_dirs = map.indexes2directions(shortest_indexes, diag_enabled);
    EXPECT_FALSE(shortest_dirs.empty());
    StepMap::appendStraightDirections(maze, shortest_dirs, known_only,
                                      diag_enabled);
    maze.print(shortest_dirs);
  }
}

TEST(CLRobotBase, fake) {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/data/";
  const std::string filename = "32_fake.maze";
  Maze maze_target;
  EXPECT_TRUE(maze_target.parse((mazedata_dir + filename).c_str()));
  CLRobotBase robot(maze_target);
  robot.replaceGoals(maze_target.getGoals());

  /* Search Run */
  robot.resetLastWalls();
  EXPECT_FALSE(robot.searchRun());
  robot.printInfo();
  robot.printSearchResult();
  for (const auto diag_enabled : {false, true}) {
    EXPECT_FALSE(robot.calcShortestDirections(diag_enabled));
    EXPECT_FALSE(robot.endFastRunBackingToStartRun());
  }

  /* Other Run */
  EXPECT_FALSE(robot.searchRun());
  EXPECT_FALSE(robot.positionIdentifyRun());
  EXPECT_FALSE(robot.isSolvable());
  EXPECT_FALSE(robot.isComplete());

  /* StepMap */
  for (const auto simple : {true, false}) {
    const bool known_only = 0;
    const Maze& maze = maze_target;
    StepMap map;
    auto shortest_dirs = map.calcShortestDirections(
        maze, maze.getStart(), maze.getGoals(), known_only, simple);
    EXPECT_TRUE(shortest_dirs.empty());
    map.appendStraightDirections(maze, shortest_dirs, known_only, false);
    map.print(maze, shortest_dirs);
    map.print(maze);
    map.printFull(maze, shortest_dirs);
    map.printFull(maze);
    maze.print(shortest_dirs);
  }

  /* StepMapWall */
  for (const auto simple : {true, false}) {
    const bool known_only = 0;
    const Maze& maze = maze_target;
    StepMapWall map;
    auto shortest_dirs = map.calcShortestDirections(maze, known_only, simple);
    EXPECT_TRUE(shortest_dirs.empty());
    map.appendStraightDirections(maze, shortest_dirs);
    map.print(maze, shortest_dirs);
    map.print(maze);
    maze.print(StepMapWall::convertWallIndexDirectionsToPositionDirections(
                   shortest_dirs, WallIndex(Position(0, 0), Direction::North)),
               maze.getStart());
  }

  /* StepMapSlalom */
  for (const auto diag_enabled : {false, true}) {
    const bool known_only = 0;
    const Maze& maze = maze_target;
    StepMapSlalom map;
    StepMapSlalom::Indexes shortest_indexes;
    map.update(maze, StepMapSlalom::EdgeCost(),
               StepMapSlalom::convertDestinations(maze.getGoals()), known_only,
               diag_enabled);
    EXPECT_FALSE(map.genPathFromMap(shortest_indexes));
    map.print(maze, shortest_indexes);
    auto shortest_dirs = map.indexes2directions(shortest_indexes, diag_enabled);
    EXPECT_TRUE(shortest_dirs.empty());
    StepMap::appendStraightDirections(maze, shortest_dirs, known_only,
                                      diag_enabled);
    maze.print(shortest_dirs);
  }
}
