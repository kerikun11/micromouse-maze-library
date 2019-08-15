#include "CLRobotBase.h"

using namespace MazeLib;

class CLRobot : public CLRobotBase {
public:
  CLRobot(const Maze &maze_target) : CLRobotBase(maze_target) {}
  bool display = false;
  void printInfo() {
    CLRobotBase::printInfo();
    std::cout << "P.I. wall:\t"
              << getSearchAlgorithm().getIdMaze().getWallLogs().size() << "    "
              << std::endl;
  }

protected:
  virtual void
  calcNextDirsPostCallback(SearchAlgorithm::State prevState,
                           SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirsPostCallback(prevState, newState);
    if (newState == prevState)
      return;
    /* State Change has occurred */
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      display = 0;
      // printInfo();
      // std::cout << "Fake:\t" << fake_offset << std::endl;
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
    }
#if 1
    if (getState() == SearchAlgorithm::IDENTIFYING_POSITION &&
        real.first == maze.getStart())
      logw << "Visited Start! fake_offset: " << fake_offset << std::endl;
#endif
    CLRobotBase::queueAction(action);
  }
};

int test_position_identify() {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/";
  const std::string filename = "32MM2014HX.maze";
  Maze maze_target = Maze((mazedata_dir + filename).c_str());
  const auto p_robot = std::make_unique<CLRobot>(maze_target);
  CLRobot &robot = *p_robot;
  robot.replaceGoals(maze_target.getGoals());
  robot.searchRun();

#if 0
  /* Position Identification Run */
  robot.display = 1;
  robot.fake_offset.second = robot.real.second = Dir::South;
  robot.fake_offset.first = robot.real.first = Vector(0, 2);
  bool res = robot.positionIdentifyRun();
  if (!res) {
    robot.printInfo();
    std::cout << std::endl
              << "Failed to Identify! offset:\t"
              << VecDir{robot.fake_offset.first, robot.fake_offset.second}
              << std::endl;
    getc(stdin);
  }
#endif

#if 1
  /* Position Identification Run */
  StepMap stepMap;
  stepMap.update(maze_target, maze_target.getGoals(), false, false);
  for (int8_t x = 0; x < MAZE_SIZE; ++x)
    for (int8_t y = 0; y < MAZE_SIZE; ++y)
      for (const auto d : Dir::ENWS()) {
        const auto v = Vector(x, y);
        if (stepMap.getStep(v) == MAZE_STEP_MAX)
          continue;
        if (v == Vector(0, 0) || v == Vector(0, 1))
          continue;
        robot.real.first = robot.fake_offset.first = Vector(x, y);
        robot.real.second = robot.fake_offset.second = d;
        robot.display = 1;
        bool res = robot.positionIdentifyRun();
        if (!res) {
          robot.printInfo();
          std::cout << std::endl
                    << "Failed to Identify! offset:\t"
                    << VecDir{robot.fake_offset.first, robot.fake_offset.second}
                    << std::endl;
          getc(stdin);
        }
      }
  std::cout << "P.I. Max Time:\t" << robot.max_usec << "\t[us]" << std::endl;
#endif
  std::cout << std::endl << "End" << std::endl;

  return 0;
}

int main(void) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  return test_position_identify();
}
