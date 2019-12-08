#include "CLRobotBase.h"

using namespace MazeLib;

class CLRobot : public CLRobotBase {
public:
  CLRobot(Maze &maze_target) : CLRobotBase(maze_target) {}
  bool display = false;

protected:
  virtual void
  calcNextDirectionsPostCallback(SearchAlgorithm::State prevState,
                                 SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirectionsPostCallback(prevState, newState);
    if (newState == prevState)
      return;
    /* State Change has occurred */
    if (prevState == SearchAlgorithm::SEARCHING_FOR_GOAL) {
      // display = 1;
    }
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      // display = 1;
    }
  }
  virtual void crashed() override {
    printInfo();
    CLRobotBase::crashed();
    getc(stdin);
  }
  virtual void queueAction(const Action action) override {
    if (display) {
      printInfo();
      getc(stdin);
    }
    CLRobotBase::queueAction(action);
    /* fix mistook wall */
    if (step == 354) {
      maze_target.setWall(0, 30, Direction::East, false);
    }
    /* 1st crashed here */
    if (step == 1071) {
      setBreakFlag();
    }
    /* 1st timeout here */
    if (step == 1387) {
      setForceBackToStart();
    }
    /* 2nd timeout here */
    if (step == 1660) {
      setForceBackToStart();
    }
  }
};

int main(void) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/";
  const std::string filename = "32MM2019HX.maze";
  Maze maze_target = Maze((mazedata_dir + filename).c_str());
  const auto p_robot = std::make_unique<CLRobot>(maze_target);
  CLRobot &robot = *p_robot;
  robot.replaceGoals(maze_target.getGoals());
  /* Set Mistook Wall */
  maze_target.setWall(0, 28, Direction::East, true);
  maze_target.setWall(0, 29, Direction::East, true);
  maze_target.setWall(0, 30, Direction::East, true);
  /* Search Run */
  // robot.display = 1;
  robot.searchRun();
  /* Crashed */
  {
    auto m = robot.getMaze();
    m.resetLastWalls(12 + 2);
    robot.setMaze(m);
  }
#if 1
  /* Crashed */
  // robot.display = 1;
  robot.fake_offset = robot.real = Pose(Position(23, 11), Direction::South);
  bool res = robot.positionIdentifyRun(false);
  if (!res) {
    robot.printInfo();
    std::cout << std::endl
              << "Failed to Identify! offset:\t" << robot.fake_offset
              << std::endl;
    getc(stdin);
  }
#endif
  /* Crashed */
  {
    auto m = robot.getMaze();
    m.resetLastWalls(12);
    robot.setMaze(m);
  }
#if 1
  /* Crashed */
  {
    maze_target.setWall(27, 1, Direction::North, true);
    // robot.display = 1;
    robot.fake_offset = robot.real = Pose(Position(2, 1), Direction::East);
    bool res = robot.positionIdentifyRun(false);
    if (!res) {
      robot.printInfo();
      std::cout << std::endl
                << "Failed to Identify! offset:\t" << robot.fake_offset
                << std::endl;
      getc(stdin);
    }
  }
#endif
#if 1
  /* WallLogs */
  std::cout << std::endl;
  for (const auto &wl : robot.getMaze().getWallLogs())
    std::cout << wl << std::endl;
#endif
#if 0
  /* Result */
  robot.updateCurrentPose({Position(0, 1), Direction::South});
  robot.printInfo();
  robot.fastRun(false);
  robot.printPath();
  robot.fastRun(true);
  robot.printPath();
#endif

  return 0;
}
