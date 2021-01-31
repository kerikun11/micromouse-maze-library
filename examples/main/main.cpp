#include "CLRobotBase.h"

#include <thread>

using namespace MazeLib;

#if 1

std::ofstream csv("main.csv");

class CLRobot : public CLRobotBase {
public:
  CLRobot(Maze &maze_target) : CLRobotBase(maze_target) {}
  bool display = 0;

protected:
  virtual void
  calcNextDirectionsPostCallback(SearchAlgorithm::State prevState,
                                 SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirectionsPostCallback(prevState, newState);
    csv << t_dur << std::endl;
    if (newState == prevState)
      return;
    /* State Change has occurred */
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
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#if 0
    /* 袋小路以外の転回検出 */
    if (action == SearchAction::ROTATE_180 &&
        maze_target.wallCount(real.p) != 3)
      maze_logi << "it's not dead end!" << std::endl, getc(stdin);
#endif
    CLRobotBase::queueAction(action);
  }
};

#endif

int main(int argc, char *argv[]) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);

#if 1
  /* Preparation */
  const std::string filepath =
      argc > 1 ? std::string(argv[1]) : "../mazedata/data/32MM2019HX.maze";
  Maze maze_target;
  if (!maze_target.parse(filepath))
    return -1;
  const auto p_robot = std::make_unique<CLRobot>(maze_target);
  CLRobot &robot = *p_robot;
  robot.replaceGoals(maze_target.getGoals());
#endif

#if 1
  /* Search Run */
  robot.display = 1;
  robot.searchRun();
#endif

#if 0
  /* Fast Run */
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
  for (bool diag_enabled : {false, true}) {
    robot.calcShortestDirections(diag_enabled);
    robot.printPath();
    std::cout << "Estimated Shortest Time "
              << (diag_enabled ? "(diag)" : "(no diag)") << ": "
              << robot.getSearchAlgorithm().getShortestCost() << "\t[ms]"
              << std::endl;
  }
#endif

#if 0
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
        maze_loge << "Failed!" << std::endl;
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
        maze_loge << "Failed!" << std::endl;
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

#if 0
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
    // StepMap::appendStraightDirections(maze, shortest_dirs, known_only,
    //                                   diag_enabled);
    // maze.print(shortest_dirs);
  }
#endif

  return 0;
}
