#include "MazeLib/CLRobotBase.h"

#include <gtest/gtest.h>

using namespace MazeLib;

TEST(CLRobotBase, CLRobotBase) {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/data/";
  // const std::string filename = "32MM2019HX.maze";
  const std::string filename = "16MM2020CX.maze";
  // const std::string filename = "09MM2019C_Cheese_cand.maze";
  Maze mazeTarget;
  EXPECT_TRUE(mazeTarget.parse((mazedata_dir + filename).c_str()));
  CLRobotBase robot(mazeTarget);
  robot.replaceGoals(mazeTarget.getGoals());

  /* Search Run */
  robot.resetLastWalls();
  EXPECT_TRUE(robot.isSolvable());
  EXPECT_FALSE(robot.isCompleted());
  EXPECT_TRUE(robot.searchRun());
  robot.printInfo();
  robot.printSearchResult();
  for (const auto diagEnabled : {false, true}) {
    EXPECT_TRUE(robot.calcShortestDirections(diagEnabled));
    EXPECT_TRUE(robot.getSearchAlgorithm().getShortestCost());
    EXPECT_TRUE(robot.fastRun(diagEnabled));
    robot.printPath();
  }

  /* Other Run */
  EXPECT_TRUE(robot.searchRun());
  EXPECT_TRUE(robot.positionIdentifyRun());

  /* StepMap */
  for (const auto simple : {true, false}) {
    const bool knownOnly = 0;
    const Maze& maze = mazeTarget;
    StepMap map;
    auto shortestDirections = map.calcShortestDirections(
        maze, maze.getStart(), maze.getGoals(), knownOnly, simple);
    EXPECT_FALSE(shortestDirections.empty());
    map.appendStraightDirections(maze, shortestDirections, knownOnly, false);
    map.print(maze, shortestDirections);
    map.print(maze);
    map.printFull(maze, shortestDirections);
    map.printFull(maze);
    maze.print(shortestDirections);
  }

  /* StepMapWall */
  for (const auto simple : {true, false}) {
    const bool knownOnly = 0;
    const Maze& maze = mazeTarget;
    StepMapWall map;
    auto shortestDirections =
        map.calcShortestDirections(maze, knownOnly, simple);
    EXPECT_FALSE(shortestDirections.empty());
    map.appendStraightDirections(maze, shortestDirections);
    map.print(maze, shortestDirections);
    map.printPath(maze, shortestDirections);
    maze.print(StepMapWall::convertWallIndexDirectionsToPositionDirections(
        shortestDirections));
  }

  /* StepMapSlalom */
  const bool knownOnly = 0;
  const Maze& maze = mazeTarget;
  StepMapSlalom map;
  StepMapSlalom::Indexes shortestIndexes;
  map.update(maze, StepMapSlalom::EdgeCost(),
             StepMapSlalom::convertDestinations(maze.getGoals()), knownOnly);
  EXPECT_TRUE(map.genPathFromMap(shortestIndexes));
  map.print(maze, shortestIndexes);
  auto shortestDirections = map.indexes2directions(shortestIndexes);
  EXPECT_FALSE(shortestDirections.empty());
  StepMap::appendStraightDirections(maze, shortestDirections, knownOnly, true);
  maze.print(shortestDirections);
}

TEST(CLRobotBase, fake) {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/data/";
  const std::string filename = "32_fake.maze";
  Maze mazeTarget;
  EXPECT_TRUE(mazeTarget.parse((mazedata_dir + filename).c_str()));
  CLRobotBase robot(mazeTarget);
  robot.replaceGoals(mazeTarget.getGoals());

  /* Search Run */
  robot.resetLastWalls();
  EXPECT_FALSE(robot.searchRun());
  robot.printInfo();
  robot.printSearchResult();
  for (const auto diagEnabled : {false, true}) {
    EXPECT_FALSE(robot.calcShortestDirections(diagEnabled));
    EXPECT_FALSE(robot.endFastRunBackingToStartRun());
  }

  /* Other Run */
  EXPECT_FALSE(robot.searchRun());
  EXPECT_FALSE(robot.positionIdentifyRun());
  EXPECT_FALSE(robot.isSolvable());
  EXPECT_FALSE(robot.isCompleted());

  /* StepMap */
  for (const auto simple : {true, false}) {
    const bool knownOnly = 0;
    const Maze& maze = mazeTarget;
    StepMap map;
    auto shortestDirections = map.calcShortestDirections(
        maze, maze.getStart(), maze.getGoals(), knownOnly, simple);
    EXPECT_TRUE(shortestDirections.empty());
    map.appendStraightDirections(maze, shortestDirections, knownOnly, false);
    map.print(maze, shortestDirections);
    map.print(maze);
    map.printFull(maze, shortestDirections);
    map.printFull(maze);
    maze.print(shortestDirections);
  }

  /* StepMapWall */
  for (const auto simple : {true, false}) {
    const bool knownOnly = 0;
    const Maze& maze = mazeTarget;
    StepMapWall map;
    auto shortestDirections =
        map.calcShortestDirections(maze, knownOnly, simple);
    EXPECT_TRUE(shortestDirections.empty());
    map.appendStraightDirections(maze, shortestDirections);
    map.print(maze, shortestDirections);
    map.printPath(maze, shortestDirections);
    maze.print(StepMapWall::convertWallIndexDirectionsToPositionDirections(
                   shortestDirections),
               maze.getStart());
  }

  /* StepMapSlalom */
  const bool knownOnly = 0;
  const Maze& maze = mazeTarget;
  StepMapSlalom map;
  StepMapSlalom::Indexes shortestIndexes;
  map.update(maze, StepMapSlalom::EdgeCost(),
             StepMapSlalom::convertDestinations(maze.getGoals()), knownOnly);
  EXPECT_FALSE(map.genPathFromMap(shortestIndexes));
  map.print(maze, shortestIndexes);
  auto shortestDirections = map.indexes2directions(shortestIndexes);
  EXPECT_TRUE(shortestDirections.empty());
  StepMap::appendStraightDirections(maze, shortestDirections, knownOnly, true);
  maze.print(shortestDirections);
}
