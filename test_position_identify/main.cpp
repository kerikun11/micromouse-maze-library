#include "CLRobotBase.h"

using namespace MazeLib;

class CLRobot : public CLRobotBase {
public:
  CLRobot(const Maze &maze_target) : CLRobotBase(maze_target) {}
  bool display = false;

protected:
  virtual void
  calcNextDirsPostCallback(SearchAlgorithm::State prevState,
                           SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirsPostCallback(prevState, newState);
    if (newState == prevState)
      return;
    /* State Change has occurred */
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      if (display)
        printInfo();
      display = 0;
    }
  }
  virtual void crashed() override {
    printInfo();
    CLRobotBase::crashed();
    getc(stdin);
  }
  virtual void queueAction(const Action action) override {
    if (display)
      printInfo();
    if (getState() == SearchAlgorithm::IDENTIFYING_POSITION &&
        real.first == maze.getStart())
      logw << "Visited Start! fake_offset: " << fake_offset << std::endl;
    CLRobotBase::queueAction(action);
  }
};

void test_position_identify() {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/";
  const std::string filename = mazedata_dir + "32MM2018HX.maze";
  Maze maze_target = Maze(filename.c_str());
  const auto p_robot = std::unique_ptr<CLRobot>(new CLRobot(maze_target));
  CLRobot &robot = *p_robot;
  robot.replaceGoals(maze_target.getGoals());
  robot.searchRun();

#if 1
  /* Position Identification Run */
  robot.display = 1;
  robot.fake_offset.second = robot.real.second = Dir::South;
  robot.fake_offset.first = robot.real.first = Vector(31, 2);
  bool res = robot.positionIdentifyRun(robot.real.second);
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
  stepMap.updateSimple(maze_target, maze_target.getGoals(), false);
  for (int8_t x = 0; x < MAZE_SIZE; ++x)
    for (int8_t y = 0; y < MAZE_SIZE; ++y)
      for (const auto ed : Dir::ENWS()) {
        const auto v = Vector(x, y);
        if (stepMap.getStep(v) == MAZE_STEP_MAX)
          continue;
        if (v == Vector(0, 0) || v == Vector(0, 1))
          continue;
        // if (maze_target.isWall(v, ed))
        //   continue;
        robot.real.first = robot.fake_offset.first = Vector(x, y);
        robot.real.second = robot.fake_offset.second = ed;
        robot.display = 1;
        bool res = robot.positionIdentifyRun(ed);
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
}

int main(void) {
  test_position_identify();
  return 0;
}
