#include "CLRobotBase.h"

using namespace MazeLib;

class CLRobot : public CLRobotBase {
public:
  CLRobot(const Maze &maze_target) : CLRobotBase(maze_target) {}
  bool display = 0;
  void printInfo() {
    CLRobotBase::printInfo();
    std::cout << "P.I. wall:\t"
              << getSearchAlgorithm().getIdMaze().getWallLogs().size() << "    "
              << std::endl;
  }

protected:
  virtual void
  calcNextDirectionsPostCallback(SearchAlgorithm::State prevState,
                                 SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirectionsPostCallback(prevState, newState);
    if (newState == prevState)
      return;
    /* state change has occurred */
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      // display = 0;
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
#if 1
    if (getState() == SearchAlgorithm::IDENTIFYING_POSITION &&
        real.p == maze.getStart() && action != ST_HALF_STOP)
      logw << "Visited Start! fake_offset: " << fake_offset << std::endl;
#endif
    CLRobotBase::queueAction(action);
  }
};

int test_position_identify() {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/";
  // const std::string filename = "32MM2013HX.maze";
  // const std::string filename = "16MM2019H_kansai.maze";
  const std::string filename = "16MM2019H_kanazawa.maze";
  Maze maze_target = Maze((mazedata_dir + filename).c_str());
  const auto p_robot = std::make_unique<CLRobot>(maze_target);
  CLRobot &robot = *p_robot;
  robot.replaceGoals(maze_target.getGoals());
  robot.searchRun();
  // robot.printInfo();

#if 1
  /* Position Identification Run */
  robot.display = 1;
  robot.fake_offset = robot.real = Pose(Position(1, 1), Direction::East);
  bool res = robot.positionIdentifyRun();
  if (!res) {
    robot.printInfo();
    std::cout << std::endl
              << "Failed to Identify! offset:\t" << robot.fake_offset
              << std::endl;
    getc(stdin);
  }
#endif

#if 0
  /* Position Identification Run */
  StepMap step_map;
  step_map.update(maze_target, maze_target.getGoals(), false, false);
  for (int8_t x = 0; x < MAZE_SIZE; ++x)
    for (int8_t y = 0; y < MAZE_SIZE; ++y)
      for (const auto d : Direction::getAlong4()) {
        const auto p = Position(x, y);
        if (step_map.getStep(p) == STEP_MAX)
          continue;
        if (p == Position(0, 0) || p == Position(0, 1))
          continue;
        robot.fake_offset = robot.real = Pose(Position(x, y), d);
        robot.display = 1;
        bool res = robot.positionIdentifyRun();
        if (!res) {
          robot.printInfo();
          std::cout << std::endl
                    << "Failed to Identify! offset:\t" << robot.fake_offset
                    << std::endl;
          getc(stdin);
        }
      }
  std::cout << "P.I. Max Time:\t" << robot.t_dur_max << "\t[us]" << std::endl;
#endif
  std::cout << std::endl << "End" << std::endl;

  return 0;
}

int main(void) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  return test_position_identify();
}
