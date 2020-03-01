#include "CLRobotBase.h"
#include "ShortestAlgorithm.h"

#include <thread>

using namespace MazeLib;

#if 1

std::ofstream csv("main.csv");

class CLRobot : public CLRobotBase {
public:
  CLRobot(Maze &maze_target) : CLRobotBase(maze_target) {}
  bool display = false;
  bool continue_straight_if_no_front_wall = false;
  bool continue_straight_if_no_front_wall_prev = false;

protected:
  virtual void
  calcNextDirectionsPostCallback(SearchAlgorithm::State prevState,
                                 SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirectionsPostCallback(prevState, newState);
    const auto d = !getNextDirections().empty() ? getNextDirections().back()
                                                : current_pose.d;
    continue_straight_if_no_front_wall_prev =
        continue_straight_if_no_front_wall;
    continue_straight_if_no_front_wall =
        newState != SearchAlgorithm::GOING_TO_GOAL &&
        newState != SearchAlgorithm::IDENTIFYING_POSITION &&
        !getNextDirectionCandidates().empty() &&
        getNextDirectionCandidates()[0] == d;
#if 0
    /* 既知区間観測用 */
    if (getNextDirections().size() > 0) {
      printInfo();
      getc(stdin);
    }
#endif
    csv << t_dur << std::endl;
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
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#if 0
    /* 未知区間加速のバグ探し */
    if (continue_straight_if_no_front_wall_prev &&
        getNextDirections().empty() &&
        !maze.isWall(current_pose.p, current_pose.d) &&
        action != Action::ST_FULL) {
      printInfo();
      logw << std::endl;
      getc(stdin);
    }
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
  const std::string filename = "32MM2019HX.maze";
  Maze maze_target = Maze((mazedata_dir + filename).c_str());
  const auto p_robot = std::make_unique<CLRobot>(maze_target);
  CLRobot &robot = *p_robot;
  robot.replaceGoals(maze_target.getGoals());
#endif

#if 1
  /* Search Run */
  robot.display = 1;
  robot.searchRun();
  robot.updateCurrentPose({Position(0, 1), Direction::South});
  robot.fastRun(false);
  robot.endFastRunBackingToStartRun();
  robot.fastRun(true);
  robot.endFastRunBackingToStartRun();
#endif

#if 1
  /* Show Result */
  std::printf("Estimated Search Time: %2d:%02d, Step: %4d, Forward: %3d, "
              "Left: %3d, Right: %3d, Back: %3d\n",
              ((int)robot.cost / 60) % 60, ((int)robot.cost) % 60, robot.step,
              robot.f, robot.l, robot.r, robot.b);
  for (bool diag_enabled : {true, false}) {
    robot.calcShortestDirections(diag_enabled);
    robot.printPath();
    std::cout << "Estimated Shortest Time "
              << (diag_enabled ? "(diag)" : "(no diag)") << ": "
              << robot.getSearchAlgorithm().getShortestCost() << "\t[ms]"
              << std::endl;
  }
#endif

#if 0
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
    sa.print(path);
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
    // map.print(maze, shortest_dirs);
    maze.print(shortest_dirs);
  }
#endif

#if 0
  /* StepMapDijkstra */
  for (const auto simple : {true, false}) {
    const bool known_only = 0;
    const Maze &maze = maze_target;
    const auto p = std::make_unique<StepMap>();
    StepMap &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 100;
    Positions shortest_indexes;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      shortest_indexes =
          map.calcShortestDirectionsDijkstra(maze, known_only, simple);
      if (shortest_indexes.empty())
        loge << "Failed!" << std::endl;
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepMapDi " << (simple ? "s" : "n") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
    // maze.print(shortest_indexes);
  }
#endif

#if 0
  /* BFS */
  for (const auto simple : {true, false}) {
    const bool known_only = 0;
    const Maze &maze = maze_target;
    const auto p = std::make_unique<StepMap>();
    StepMap &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 100;
    Positions shortest_indexes;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      shortest_indexes =
          map.calcShortestDirectionsBFS(maze, known_only, simple);
      if (shortest_indexes.empty())
        loge << "Failed!" << std::endl;
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepMapBFS " << (simple ? "s" : "n") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
    maze.print(shortest_indexes);
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
    std::cout << "StepSla " << (diag_enabled ? "diag" : "no_d") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
    map.print(maze, path);
    // auto shortest_dirs = map.indexes2directions(path, diag_enabled);
    // Maze::appendStraightDirections(maze, shortest_dirs, diag_enabled);
    // maze.print(shortest_dirs);
  }
#endif

  return 0;
}
