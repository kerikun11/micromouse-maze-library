#include "MazeLib/CLRobotBase.h"

#include <gtest/gtest.h>

using namespace MazeLib;

class Robot : public RobotBase {
 public:
  Robot(Maze& maze_target) : maze_target(maze_target) {
    replaceGoals(maze_target.getGoals());
  }

 public:
  Maze& maze_target;

  virtual void senseWalls(bool& left, bool& front, bool& right) override {
    const auto& pose = getCurrentPose();
    left = !maze_target.canGo(pose.p, pose.d + Direction::Left);
    front = !maze_target.canGo(pose.p, pose.d + Direction::Front);
    right = !maze_target.canGo(pose.p, pose.d + Direction::Right);
  }
};

TEST(RobotBase, RobotBase) {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/data/";
  const std::string filename = "09MM2019C_Cheese_cand.maze";
  Maze maze_target;
  EXPECT_TRUE(maze_target.parse((mazedata_dir + filename).c_str()));
  Robot robot(maze_target);
  robot.replaceGoals(maze_target.getGoals());

  /* Search Run */
  EXPECT_TRUE(robot.searchRun());
}
