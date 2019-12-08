#include "CLRobotBase.h"
#include <thread>

using namespace MazeLib;

class CLRobot : public CLRobotBase {
public:
  CLRobot(Maze &maze_target) : CLRobotBase(maze_target) {}
  bool display = false;

  void wait() {
    if (!display)
      return;
    // getc(stdin);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

protected:
  virtual void
  calcNextDirectionsPostCallback(SearchAlgorithm::State prevState,
                                 SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirectionsPostCallback(prevState, newState);
    if (newState == prevState)
      return;
    /* State Change has occurred */
    if (prevState == SearchAlgorithm::SEARCHING_FOR_GOAL) {
    }
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      if (display) {
        wait();
        printInfo();
        wait();
      }
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
      // getc(stdin);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    /* fix mistook wall */
    if (step == 354) {
      maze_target.setWall(0, 28, Direction::East, false);
      maze_target.setWall(0, 29, Direction::East, false);
      maze_target.setWall(0, 30, Direction::East, false);
    }
    /* 1st crashed here */
    if (step == 1071) {
      setBreakFlag();
      wait();
    }
    /* 1st timeout here */
    if (step == 1347) {
      setForceBackToStart();
      wait();
    }
    /* 2nd crashed here */
    if (step == 1502) {
      setBreakFlag();
      setForceBackToStart();
      wait();
    }
    CLRobotBase::queueAction(action);
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
  robot.display = 1;
  robot.searchRun();
  /* 1st Crash */
  {
    auto m = robot.getMaze();
    m.resetLastWalls(12);
    robot.setMaze(m);
  }

  /* 1st Recovery */
  robot.fake_offset = robot.real = Pose(Position(23, 11), Direction::South);
  robot.positionIdentifyRun(false);

  /* Reset Last Wall */
  {
    auto m = robot.getMaze();
    m.resetLastWalls(12);
    robot.setMaze(m);
  }
  robot.wait();

  /* 1st Fast Run */
  robot.fastRun(true);
  /* 2nd Recovery */
  /* Set Mistook Wall */
  maze_target.setWall(27, 1, Direction::North, true);
  robot.fake_offset = robot.real = Pose(Position(2, 1), Direction::East);
  robot.setForceBackToStart();
  robot.positionIdentifyRun(false);

  /* 2nd Fast Run */
  robot.wait();
  robot.fastRun(false);
  robot.wait();
  robot.setForceBackToStart();
  robot.endFastRunBackingToStartRun();

  /* 3rd Fast Run */
  robot.wait();
  robot.fastRun(false);
  robot.wait();
  robot.setForceBackToStart();
  robot.endFastRunBackingToStartRun();

#if 0
  /* Result */
  robot.updateCurrentPose({Position(0, 1), Direction::South});
  robot.printInfo();
  robot.fastRun(false);
  robot.printPath();
  robot.fastRun(true);
  robot.printPath();
#endif
#if 0
  /* WallLogs */
  std::cout << std::endl;
  for (const auto &wl : robot.getMaze().getWallLogs())
    std::cout << wl << std::endl;
#endif

  return 0;
}
