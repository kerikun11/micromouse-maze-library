#include "CLRobotBase.h"
#include "ShortestAlgorithm.h"

using namespace MazeLib;

#if 1

std::ofstream csv("main.csv");

class CLRobot : public CLRobotBase {
public:
  CLRobot(const Maze &maze_target) : CLRobotBase(maze_target) {}
  bool display = false;

protected:
  virtual void
  calcNextDirectionsPostCallback(SearchAlgorithm::State prevState,
                                 SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirectionsPostCallback(prevState, newState);
#if 0
    /* 既知区間観測用 */
    if (getNextDirections().size() > 0) {
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
        real.p == maze.getStart())
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
  const auto p_robot = std::make_unique<CLRobot>(maze_target);
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

#if 1
  /* Shortest Algorithm */
  for (const auto diag_enabled : {false, true}) {
    const int n = 100;
    const bool known_only = 0;
    const Maze &maze = maze_target;
    const auto p_sa = std::make_unique<ShortestAlgorithm>(maze);
    ShortestAlgorithm &sa = *p_sa;
    Indexes path;
    std::chrono::microseconds sum{0};
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      if (!sa.calcShortestPath(path, known_only, diag_enabled))
        loge << "Failed!" << std::endl;
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "Shortest " << (diag_enabled ? "diag" : "no_d") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
    std::cout << "PathCost " << (diag_enabled ? "diag" : "no_d") << ":\t"
              << sa.getShortestPathCost() << "\t[ms]" << std::endl;
    // sa.print(path);
  }
#endif

#if 1
  /* StepMap */
  for (const auto simple : {true, false}) {
    const bool known_only = 0;
    const Maze &maze = maze_target;
    const auto p = std::make_unique<StepMap>();
    StepMap &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 100;
    Directions shortest_dirs;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      shortest_dirs = map.calcShortestDirections(
          maze, maze.getStart(), maze.getGoals(), known_only, simple);
      if (shortest_dirs.empty())
        loge << "Failed!" << std::endl;
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepMap " << (simple ? "simple" : "normal") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
    map.print(maze, shortest_dirs);
  }
#endif

#if 1
  /* StepMapWall */
  for (const auto simple : {true, false}) {
    const bool known_only = 0;
    const Maze &maze = maze_target;
    const auto p = std::make_unique<StepMapWall>();
    StepMapWall &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 100;
    Directions shortest_dirs;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      shortest_dirs = map.calcShortestDirections(maze, known_only, simple);
      if (shortest_dirs.empty())
        loge << "Failed!" << std::endl;
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepMapWall " << (simple ? "s" : "n") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
    map.print(maze, shortest_dirs);
  }
#endif

#if 1
  /* StepMapSlalom */
  for (const auto diag_enabled : {false, true}) {
    const bool known_only = 0;
    const Maze &maze = maze_target;
    const auto p = std::make_unique<StepMapSlalom>();
    StepMapSlalom &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 100;
    StepMapSlalom::Indexes path;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      map.update(maze, StepMapSlalom::EdgeCost(),
                 StepMapSlalom::convertDestinations(maze.getGoals()),
                 known_only, diag_enabled);
      map.genPathFromMap(path);
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepMapSlalom\t" << sum.count() / n << "\t[us]" << std::endl;
    map.print(maze, path);
    auto shortest_dirs = map.indexes2dirs(path, diag_enabled);
    Maze::appendStraightDirections(maze, shortest_dirs, diag_enabled);
    maze.print(shortest_dirs);
  }
#endif

  return 0;
}
