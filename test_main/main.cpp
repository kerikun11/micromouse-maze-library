#include "CLRobotBase.h"

using namespace MazeLib;

#if 1

std::ofstream csv("../matlab/out.csv");

class CLRobot : public CLRobotBase {
public:
  CLRobot(const Maze &maze_target) : CLRobotBase(maze_target) {}
  bool display = false;

protected:
  virtual void
  calcNextDirsPostCallback(SearchAlgorithm::State prevState,
                           SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirsPostCallback(prevState, newState);
#if 0
    /* 既知区間観測用 */
    if (getNextDirs().size() > 0) {
      printInfo();
      getc(stdin);
    }
#endif
    csv << usec << std::endl;
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

#endif

int main(void) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);

#if 1
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/";
  const std::string filename = "32MM2016HX.maze";
  Maze maze_target = Maze((mazedata_dir + filename).c_str());
  const auto p_robot = std::unique_ptr<CLRobot>(new CLRobot(maze_target));
  CLRobot &robot = *p_robot;
  robot.replaceGoals(maze_target.getGoals());
#endif

#if 1
  /* Search Run */
  robot.display = 1;
  robot.searchRun();
  robot.printInfo();
  robot.fastRun(false);
  // robot.endFastRunBackingToStartRun();
  robot.printPath();
  robot.fastRun(true);
  robot.printPath();
  // robot.endFastRunBackingToStartRun();
#endif

#if 0
  /* D* Lite */
  const int n = 1;
  const bool diag_enabled = 1;
  const bool known_only = 0;
  std::chrono::microseconds sum{0};
  Maze maze = Maze(filename.c_str());
  ShortestAlgorithm sa(maze);
  ShortestAlgorithm::Indexes path;
  for (int i = 0; i < n; ++i) {
    const auto t_s = std::chrono::system_clock().now();
    sa.Initialize();
    sa.ComputeShortestPath(known_only, diag_enabled);
    sa.FollowShortestPath(path, known_only, diag_enabled);
    const auto t_e = std::chrono::system_clock().now();
    const auto us =
        std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
    sum += us;
  }
  sa.printPath(std::cout, path);
  std::cout << "It took " << sum.count() / n << " [us]" << std::endl;
#endif

  return 0;
}
