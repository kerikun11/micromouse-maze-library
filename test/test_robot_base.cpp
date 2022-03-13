#include "MazeLib/CLRobotBase.h"

#include <gtest/gtest.h>

using namespace MazeLib;

TEST(RobotBase, RobotBase) {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/data/";
  const std::string filename = "09MM2019C_Cheese_cand.maze";
  Maze maze_target;
  EXPECT_TRUE(maze_target.parse((mazedata_dir + filename).c_str()));
  CLRobotBase robot(maze_target);
  robot.replaceGoals(maze_target.getGoals());

  /* Search Run */
  EXPECT_TRUE(robot.isSolvable());
  EXPECT_TRUE(robot.searchRun());
  EXPECT_TRUE(robot.isComplete());
}
