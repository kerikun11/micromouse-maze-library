#include "CLRobotBase.h"

using namespace MazeLib;

#if 1

std::ofstream csv("main.csv");

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
    const int n = 1000;
    const bool known_only = 0;
    // Maze maze = Maze((mazedata_dir + filename).c_str());
    Maze maze = Maze({
        Vector(MAZE_SIZE - 2, MAZE_SIZE - 2),
        // Vector(MAZE_SIZE - 2, MAZE_SIZE - 3),
        // Vector(MAZE_SIZE - 3, MAZE_SIZE - 2),
        // Vector(MAZE_SIZE - 3, MAZE_SIZE - 3),
    });
    const auto p_sa = std::make_unique<ShortestAlgorithm>(maze);
    ShortestAlgorithm &sa = *p_sa;
    Indexes path;
    std::chrono::microseconds sum{0};
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      sa.calcShortestPath(path, known_only, diag_enabled);
      // sa.update(maze, EdgeCost(),
      //           ShortestAlgorithm::convertDestinations(maze.getGoals()),
      //           known_only, diag_enabled);
      // sa.genPathWithFromMap(path);
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "Shortest " << (diag_enabled ? "diag" : "along") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
    // sa.printPath(path);
  }
#endif

#if 0
  // StepMapWall map;
  StepMap map;
  Dirs shortest_dirs;
  // Maze &maze = maze_target;
  const Maze &maze = robot.getMaze();
  const bool known_only = false;
  // map.calcShortestDirs(maze, shortest_dirs, true, false);
  map.update(maze, maze.getGoals(), known_only, false);
  // map.print(maze, shortest_dirs);
  map.printFull(maze);
#endif

#if 0
  /* StepMap */
  for (const auto simple : {true, false}) {
    const bool known_only = 0;
    Maze maze = Maze(Vectors{Vector(MAZE_SIZE - 1, MAZE_SIZE - 1)});
    // Maze &maze = maze_target;
    const auto p = std::make_unique<StepMap>();
    StepMap &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 1000;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      map.update(maze, maze.getGoals(), known_only, simple);
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepMap::update() " << (simple ? "simple" : "normal") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
  }
#endif

#if 0
  /* StepMapWall */
  for (const auto simple : {true, false}) {
    const bool known_only = 0;
    Maze maze = Maze(Vectors{Vector(MAZE_SIZE - 1, MAZE_SIZE - 1)});
    // Maze &maze = maze_target;
    WallIndexes dest = StepMapWall::convertDestinations(maze, maze.getGoals());
    const auto p = std::make_unique<StepMapWall>();
    StepMapWall &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 1000;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      map.update(maze, dest, known_only, simple);
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepMapWall::update() " << (simple ? "simple" : "normal")
              << ":\t" << sum.count() / n << "\t[us]" << std::endl;
  }
#endif

#if 0
  /* StepMapSlalom */
  {
    const bool known_only = 0;
    Maze maze = Maze(Vectors{Vector(MAZE_SIZE - 1, MAZE_SIZE - 1)});
    // Maze &maze = maze_target;
    const auto p = std::make_unique<StepMapSlalom>();
    StepMapSlalom &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 1000;
    StepMapSlalom::Indexes path;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      map.update(maze, StepMapSlalom::EdgeCost(),
                 StepMapSlalom::convertDestinations(maze.getGoals()),
                 known_only);
      map.genPathFromMap(path);
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepMapSlalom::update() \t" << sum.count() / n << "\t[us]"
              << std::endl;
    // map.print(maze, path);
  }
#endif

  return 0;
}
