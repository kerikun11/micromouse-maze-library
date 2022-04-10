#include "MazeLib/CLRobotBase.h"

#include <thread>

using namespace MazeLib;

#if 1

std::ofstream csv("main.csv");

class CLRobot : public CLRobotBase {
 public:
  CLRobot(Maze& mazeTarget) : CLRobotBase(mazeTarget) {}
  bool display = 0;

 protected:
  virtual void calcNextDirectionsPostCallback(
      SearchAlgorithm::State oldState,
      SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirectionsPostCallback(oldState, newState);
    if (newState == oldState)
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
      getc(stdin);
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#if 0
    /* 袋小路以外の転回検出 */
    if (action == SearchAction::ROTATE_180 &&
        mazeTarget.wallCount(real.p) != 3)
      MAZE_LOGI << "it's not dead end!" << std::endl, getc(stdin);
#endif
    CLRobotBase::queueAction(action);
  }
};

#endif

int main(int argc, char* argv[]) {
#if 1
  /* Preparation */
  const std::string filepath_default = "../mazedata/data/32MM2021HX.maze";
  const std::string filepath =
      argc > 1 ? std::string(argv[1]) : filepath_default;
  Maze mazeTarget;
  if (!mazeTarget.parse(filepath))
    return -1;
  const auto p_robot = std::make_unique<CLRobot>(mazeTarget);
  CLRobot& robot = *p_robot;
  robot.replaceGoals(mazeTarget.getGoals());
#endif

#if 1
  /* Search Run */
  robot.display = 1;
  robot.searchRun();
  robot.printSearchLogs(csv);
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
  std::printf(
      "Estimated Search Time: %2d:%02d, Step: %4d, Forward: %3d, "
      "Left: %3d, Right: %3d, Back: %3d\n",
      robot.est_time / 1000 / 60 % 60, robot.est_time / 1000 % 60, robot.step,
      robot.f, robot.l, robot.r, robot.b);
  for (bool diagEnabled : {false, true}) {
    robot.calcShortestDirections(diagEnabled);
    robot.printPath();
    std::cout << "Estimated Shortest Time "
              << (diagEnabled ? "(diag)" : "(no diag)") << ": "
              << robot.getSearchAlgorithm().getShortestCost() << "\t[ms]"
              << std::endl;
  }
#endif

#if 0
  /* StepMap */
  for (const auto simple : {true, false}) {
    const bool knownOnly = 0;
    const Maze &maze = mazeTarget;
    const auto p = std::make_unique<StepMap>();
    StepMap &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 100;
    Directions shortestDirections;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      shortestDirections = map.calcShortestDirections(
          maze, maze.getStart(), maze.getGoals(), knownOnly, simple);
      if (shortestDirections.empty())
        MAZE_LOGE << "Failed!" << std::endl;
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepMap " << (simple ? "simple" : "normal") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
    // map.print(maze, shortestDirections);
    maze.print(shortestDirections);
  }
#endif

#if 0
  /* StepMapWall */
  for (const auto simple : {true, false}) {
    const bool knownOnly = 0;
    const Maze &maze = mazeTarget;
    const auto p = std::make_unique<StepMapWall>();
    StepMapWall &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 100;
    Directions shortestDirections;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      shortestDirections = map.calcShortestDirections(maze, knownOnly, simple);
      if (shortestDirections.empty())
        MAZE_LOGE << "Failed!" << std::endl;
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepMapWall " << (simple ? "s" : "n") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
    map.print(maze, shortestDirections);
  }
#endif

#if 0
  /* StepMapSlalom */
  for (const auto diagEnabled : {false, true}) {
    const bool knownOnly = 0;
    const Maze &maze = mazeTarget;
    const auto p = std::make_unique<StepMapSlalom>();
    StepMapSlalom &map = *p;
    std::chrono::microseconds sum{0};
    const int n = 100;
    StepMapSlalom::Indexes path;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      map.update(maze, StepMapSlalom::EdgeCost(),
                 StepMapSlalom::convertDestinations(maze.getGoals()),
                 knownOnly, diagEnabled);
      map.genPathFromMap(path);
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << "StepSla " << (diagEnabled ? "diag" : "no_d") << ":\t"
              << sum.count() / n << "\t[us]" << std::endl;
    map.print(maze, path);
    // auto shortestDirections = map.indexes2directions(path, diagEnabled);
    // StepMap::appendStraightDirections(maze, shortestDirections, knownOnly,
    //                                   diagEnabled);
    // maze.print(shortestDirections);
  }
#endif

  return 0;
}
