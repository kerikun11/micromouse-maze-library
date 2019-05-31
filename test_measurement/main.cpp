#include "Maze.h"
#include "RobotBase.h"
#include <chrono>
#include <cstdio>

using namespace MazeLib;

#if 1

class CLRobot : public RobotBase {
public:
  CLRobot(const Maze &maze_target)
      : RobotBase(maze), maze_target(maze_target) {}

  void printInfo(bool showMaze = true) {
    RobotBase::printInfo(showMaze);
    std::printf("Estimated Time: %2d:%02d, Step: %4d, Forward: %3d, Left: %3d, "
                "Right: %3d, Back: %3d\n",
                ((int)cost / 60) % 60, ((int)cost) % 60, step, f, l, r, b);
    std::printf("It took %5d [us], the max is %5d [us]\n", (int)usec,
                (int)max_usec);
  }
  void printResult() const {
    std::printf("Estimated Seaching Time: %2d:%02d, Step: %4d, Forward: %3d, "
                "Left: %3d, Right: %3d, Back: %3d\n",
                ((int)cost / 60) % 60, ((int)cost) % 60, step, f, l, r, b);
    std::cout << "Max Calc Time:\t" << max_usec << std::endl;
    std::cout << "Max List:\t"
              << getSearchAlgorithm().getShortestAlgorithm().max_open_list_size
              << std::endl;
    std::cout << "Max Iteration:\t"
              << getSearchAlgorithm().getShortestAlgorithm().max_iteration_size
              << std::endl;
  }

private:
  Maze maze;
  int step = 0, f = 0, l = 0, r = 0, b = 0; /**< 探索の評価のためのカウンタ */
  float cost = 0;
  int max_usec = 0;
  int usec = 0;
  std::chrono::system_clock::time_point start;
  std::chrono::system_clock::time_point end;

public:
  const Maze &maze_target;
  Vector offset_v;
  Dir offset_d;
  Vector real_v;
  Dir real_d;

  void findWall(bool &left, bool &front, bool &right, bool &back) override {
    left = maze_target.isWall(real_v, real_d + Dir::Left);
    front = maze_target.isWall(real_v, real_d + Dir::Front);
    right = maze_target.isWall(real_v, real_d + Dir::Right);
    back = maze_target.isWall(real_v, real_d + Dir::Back);
  }
  void calcNextDirsPreCallback() override {
    start = std::chrono::system_clock::now();
  }
  void calcNextDirsPostCallback(SearchAlgorithm::State prevState
                                __attribute__((unused)),
                                SearchAlgorithm::State newState
                                __attribute__((unused))) override {
    end = std::chrono::system_clock::now();
    usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start)
               .count();
    if (max_usec < usec)
      max_usec = usec;
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION)
      printInfo();
    if (newState == prevState)
      return;
    /* State Change has occurred */
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
    }
    if (newState == SearchAlgorithm::SEARCHING_ADDITIONALLY) {
    }
    if (newState == SearchAlgorithm::BACKING_TO_START) {
    }
    if (newState == SearchAlgorithm::REACHED_START) {
    }
  }
  void discrepancyWithKnownWall() override {
    // printInfo();
    std::cout << "There was a discrepancy with known information! cur:\t"
              << VecDir{getCurVec(), getCurDir()} << std::endl;
  }
  void crashed() {
    // printInfo();
    std::cerr << "The robot crashed into the wall! cur:\t"
              << VecDir{getCurVec(), getCurDir()} << std::endl;
    getc(stdin);
  }
  void queueAction(const Action action) override {
    cost += getTimeCost(action);
    step++;
    switch (action) {
    case RobotBase::START_STEP:
      real_v = Vector(0, 1);
      real_d = Dir::North;
      f++;
      break;
    case RobotBase::START_INIT:
      break;
    case RobotBase::STOP_HALF:
      break;
    case RobotBase::TURN_LEFT_90:
      real_d = real_d + Dir::Left;
      if (!maze_target.canGo(real_v, real_d))
        crashed();
      real_v = real_v.next(real_d);
      l++;
      break;
    case RobotBase::TURN_RIGHT_90:
      real_d = real_d + Dir::Right;
      if (!maze_target.canGo(real_v, real_d))
        crashed();
      real_v = real_v.next(real_d);
      r++;
      break;
    case RobotBase::ROTATE_LEFT_90:
      break;
    case RobotBase::ROTATE_RIGHT_90:
      break;
    case RobotBase::ROTATE_180:
      real_d = real_d + Dir::Back;
      if (!maze_target.canGo(real_v, real_d))
        crashed();
      real_v = real_v.next(real_d);
      b++;
      break;
    case RobotBase::STRAIGHT_FULL:
      if (!maze_target.canGo(real_v, real_d))
        crashed();
      real_v = real_v.next(real_d);
      f++;
      break;
    case RobotBase::STRAIGHT_HALF:
      break;
    }
  }
  float getTimeCost(const Action action) {
    const float velocity = 240.0f;
    const float segment = 90.0f;
    switch (action) {
    case RobotBase::START_STEP:
      return 1.0f;
    case RobotBase::START_INIT:
      return 1.0f;
    case RobotBase::STOP_HALF:
      return segment / 2 / velocity;
    case RobotBase::TURN_LEFT_90:
      return 71 / velocity;
    case RobotBase::TURN_RIGHT_90:
      return 71 / velocity;
    case RobotBase::ROTATE_LEFT_90:
      return 0.5f;
    case RobotBase::ROTATE_RIGHT_90:
      return 0.5f;
    case RobotBase::ROTATE_180:
      return 2.0f;
    case RobotBase::STRAIGHT_FULL:
      return segment / velocity;
    case RobotBase::STRAIGHT_HALF:
      return segment / 2 / velocity;
    }
    return 0;
  }
};

#endif

int main(void) {
  const std::string mazedata_dir = "../mazedata/";
  for (const auto filename : {
           mazedata_dir + "32MM2012HX.maze",
           mazedata_dir + "32MM2013HX.maze",
           mazedata_dir + "32MM2014HX.maze",
           mazedata_dir + "32MM2015HX.maze",
           mazedata_dir + "32MM2016HX.maze",
           mazedata_dir + "32MM2017HX.maze",
           mazedata_dir + "32MM2018HX.maze",
           mazedata_dir + "32MM2017CX.maze",
       }) {
    std::cout << std::endl;
    std::cout << "Maze File: \t" << filename << std::endl;

#if 1
    /* Search Run */
    Maze maze_target = Maze(filename.c_str());
    CLRobot robot(maze_target);
    robot.replaceGoals(maze_target.getGoals());
    std::chrono::microseconds sum{0};
    const int n = 1;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      robot.searchRun();
      const auto t_e = std::chrono::system_clock().now();
      const auto us =
          std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
      sum += us;
    }
    robot.printResult();
    std::cout << "Total Search:\t" << sum.count() / n << " [us]" << std::endl;
    for (const auto diag_enabled : {false, true})
      if (!robot.calcShortestDirs(diag_enabled))
        loge << "Failed to Find a Shortest Path! "
             << (diag_enabled ? "true" : "false") << std::endl;
#endif

#if 0
    /* Position Identification Run */
    for (auto diag : {true, false}) {
      robot.calcShortestDirs(diag);
      auto shortestDirs = robot.getShortestDirs();
      auto v = Vector(0, 0);
      for (const auto d : shortestDirs) {
        v = v.next(d);
        robot.real_v = robot.offset_v = v;
        for (const auto ed : Dir::ENWS()) {
          robot.real_d = robot.offset_d = ed;
          bool res = robot.positionIdentifyRun();
          if (!res) {
            // robot.printInfo();
            std::cout << std::endl
                      << "Failed to Identify! offset:\t"
                      << VecDir{robot.offset_v, robot.offset_d} << std::endl;
            // getc(stdin);
          }
        }
      }
    }
    robot.printResult();
#endif

#if 1
    /* Shortest Algorithm */
    for (const auto diag_enabled : {false, true}) {
      const int n = 100;
      const bool known_only = 0;
      Maze maze(filename.c_str());
      // Maze maze(loadMaze().getGoals());
      ShortestAlgorithm sa(maze);
      ShortestAlgorithm::Indexes path;
      std::chrono::microseconds sum{0};
      for (int i = 0; i < n; ++i) {
        const auto t_s = std::chrono::system_clock().now();
        sa.calcShortestPath(path, known_only, diag_enabled);
        const auto t_e = std::chrono::system_clock().now();
        const auto us =
            std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        sum += us;
      }
      std::cout << "Shortest " << (diag_enabled ? "diag" : "along") << ":\t"
                << sum.count() / n << " [us]" << std::endl;
      // sa.printPath(std::cout, path);
    }
#endif
  }
  std::cout << std::endl;
  return 0;
}
