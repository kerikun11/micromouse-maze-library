#include <gtest/gtest.h>

#include "MazeLib/CLRobotBase.h"

using namespace MazeLib;

class Robot : public RobotBase {
 public:
  Robot(Maze& mazeTarget) : mazeTarget(mazeTarget) {
    replaceGoals(mazeTarget.getGoals());
  }

 public:
  Maze& mazeTarget;

  virtual void senseWalls(bool& left, bool& front, bool& right) override {
    const auto& pose = getCurrentPose();
    left = !mazeTarget.canGo(pose.p, pose.d + Direction::Left);
    front = !mazeTarget.canGo(pose.p, pose.d + Direction::Front);
    right = !mazeTarget.canGo(pose.p, pose.d + Direction::Right);
  }
};

TEST(RobotBase, RobotBase) {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/data/";
  const std::string filename = "09MM2019C_Cheese_cand.maze";
  Maze mazeTarget;
  EXPECT_TRUE(mazeTarget.parse((mazedata_dir + filename).c_str()));
  Robot robot(mazeTarget);
  robot.replaceGoals(mazeTarget.getGoals());

  /* Search Run */
  EXPECT_TRUE(robot.searchRun());
}
