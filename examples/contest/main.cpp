#include <thread>

#include "MazeLib/CLRobotBase.h"

using namespace MazeLib;

class CLRobot : public CLRobotBase {
 public:
  CLRobot(Maze& mazeTarget) : CLRobotBase(mazeTarget) {}
  bool display = false;

  void wait() {
    if (!display) return;
    // getc(stdin);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

 protected:
  virtual void calcNextDirectionsPostCallback(
      SearchAlgorithm::State oldState,
      SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirectionsPostCallback(oldState, newState);
    if (newState == oldState) return;
    /* State Change has occurred */
    if (oldState == SearchAlgorithm::SEARCHING_FOR_GOAL) {
    }
    if (oldState == SearchAlgorithm::IDENTIFYING_POSITION) {
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
  virtual void queueAction(const SearchAction action) override {
    if (display) {
      printInfo();
      // getc(stdin);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
#if 1
    /* fix mistook wall */
    if (step == 354) {
      mazeTarget.setWall(0, 28, Direction::East, false);
      mazeTarget.setWall(0, 29, Direction::East, false);
      mazeTarget.setWall(0, 30, Direction::East, false);
    }
    /* 1st crashed here */
    if (step == 1071) {
      setBreakFlag();
      wait();
    }
    /* 1st timeout here */
    if (step == 1347) {
      setForceBackToStart();
      // wait();
    }
    /* 2nd crashed here */
    if (step == 1503) {
      setBreakFlag();
      wait();
    }
#endif
    CLRobotBase::queueAction(action);
  }
};

int main(void) {
  std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
  std::cout << "\e[J";    /*< カーソル以下を消去 */

  /* Preparation */
  const std::string mazedata_dir = "../mazedata/data/";
  const std::string filename = "32MM2019HX.maze";
  Maze mazeTarget;
  if (!mazeTarget.parse((mazedata_dir + filename).c_str())) return -1;
  const auto p_robot = std::make_unique<CLRobot>(mazeTarget);
  CLRobot& robot = *p_robot;
  robot.replaceGoals(mazeTarget.getGoals());

#if 1
  /* Set Mistook Wall */
  mazeTarget.setWall(0, 28, Direction::East, true);
  mazeTarget.setWall(0, 29, Direction::East, true);
  mazeTarget.setWall(0, 30, Direction::East, true);

  /* 1. Search Run */
  robot.display = 0;
  robot.searchRun();
  /* 1st Crash */
  {
    auto m = robot.getMaze();
    m.resetLastWalls(12);
    robot.updateMaze(m);
  }

  robot.display = 1;
  /* 1st Recovery */
  auto fake_offset = Pose(Position(23, 11), Direction::South);
  robot.positionIdentifyRun(fake_offset, false);

  /* Reset Last Wall */
  {
    auto m = robot.getMaze();
    m.resetLastWalls(12);
    robot.updateMaze(m);
  }
  robot.wait();

  /* 2. 1st Fast Run */
  robot.fastRun(true);
  /* 2nd Recovery */
  /* Set Mistook Wall */
  mazeTarget.setWall(27, 1, Direction::North, true);
  fake_offset = Pose(Position(2, 1), Direction::East);
  robot.setForceGoingToGoal();
  robot.setForceBackToStart();
  robot.positionIdentifyRun(fake_offset, false);

  /* 3. 2nd Fast Run */
  robot.wait();
  robot.fastRun(false);
  robot.wait();
  robot.setForceBackToStart();  //< time-up
  robot.endFastRunBackingToStartRun();

  /* 4. 3rd Fast Run */
  // robot.wait();
  // robot.fastRun(false);
  // robot.wait();
  // robot.setForceBackToStart();
  // robot.endFastRunBackingToStartRun();
#endif

#if 0
  /* Search Run */
  robot.display = 1;
  robot.searchRun();
  /* 1st Fast Run */
  robot.wait();
  robot.fastRun(false);
  robot.wait();
  robot.setForceBackToStart();
  robot.endFastRunBackingToStartRun();
  /* 1st Fast Run */
  robot.wait();
  robot.fastRun(true);
  robot.wait();
  robot.setForceBackToStart();
  robot.endFastRunBackingToStartRun();
#endif

#if 1
  /* Result */
  robot.display = 1;
  // robot.updateCurrentPose({Position(0, 1), Direction::South});
  // robot.printInfo();
  for (bool diagEnabled : {true, false}) {
    robot.calcShortestDirections(diagEnabled);
    std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
    std::cout << "\e[J";    /*< カーソル以下を消去 */
    robot.printPath();
    std::cout << "Estimated Shortest Time "
              << (diagEnabled ? "(diag)" : "(no diag)") << ": "
              << robot.getSearchAlgorithm().getShortestCost() << "\t[ms]"
              << std::endl;
    robot.wait();
  }
#endif
#if 0
  /* WallRecords */
  std::cout << std::endl;
  for (const auto &wr : robot.getMaze().getWallRecords())
    std::cout << wr << std::endl;
#endif

  return 0;
}
