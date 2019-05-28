#include "Maze.h"
#include "RobotBase.h"
#include <chrono>
#include <cstdio>

using namespace MazeLib;

#if 1

static Maze maze_target;
static bool display = 0;
static std::ofstream of("out.txt");

class CLRobot : public RobotBase {
public:
  CLRobot() : RobotBase(maze) {}

  void printInfo(const bool showMaze = true) {
    Agent::printInfo(showMaze);
    std::printf("Estimated Time: %2d:%02d, Step: %4d, Forward: %3d, Left: %3d, "
                "Right: %3d, Back: %3d\n",
                ((int)cost / 60) % 60, ((int)cost) % 60, step, f, l, r, b);
    std::printf("It took %5d [us], the max is %5d [us]\n", (int)usec,
                (int)max_usec);
  }

private:
  Maze maze;
  int step = 0, f = 0, l = 0, r = 0, b = 0; /**< 探索の評価のためのカウンタ */
  float cost = 0;
  int max_usec = 0;
  int usec = 0;
  std::chrono::system_clock::time_point start;
  std::chrono::system_clock::time_point end;

  void findWall(bool &left, bool &front, bool &right, bool &back) override {
    const auto &v = getCurVec();
    const auto &d = getCurDir();
    left = maze_target.isWall(v, d + Dir::Left);
    front = maze_target.isWall(v, d + Dir::Front);
    right = maze_target.isWall(v, d + Dir::Right);
    back = maze_target.isWall(v, d + Dir::Back);
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
    of << usec << "\t" << maze.getWallLogs().size() << std::endl;
  }
  void discrepancyWithKnownWall() override {
    printInfo();
    std::cout << "There was a discrepancy with known information! "
              << getCurVec() << " " << getCurDir() << std::endl;
  }
  void queueAction(const Action action) override {
    if (display)
      printInfo();
    cost += getTimeCost(action);
    step++;
    switch (action) {
    case RobotBase::START_STEP:
      f++;
      break;
    case RobotBase::START_INIT:
      break;
    case RobotBase::STOP_HALF:
      break;
    case RobotBase::TURN_LEFT_90:
      l++;
      break;
    case RobotBase::TURN_RIGHT_90:
      r++;
      break;
    case RobotBase::ROTATE_LEFT_90:
      break;
    case RobotBase::ROTATE_RIGHT_90:
      break;
    case RobotBase::ROTATE_180:
      b++;
      break;
    case RobotBase::STRAIGHT_FULL:
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

const Maze loadMaze() {
  switch (MAZE_SIZE) {
  case 8:
    return Maze("../mazedata/08MM2016CF_pre.maze");
  case 16:
    return Maze("../mazedata/16MM2016CX.maze");
  case 32:
    return Maze("../mazedata/32MM2016HX.maze");
  }
}

int main(void) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
#if 1
  maze_target = loadMaze();
  CLRobot robot;
  robot.replaceGoals(maze_target.getGoals());
  display = 0;
  robot.searchRun();
  robot.printInfo();
  robot.fastRun(false);
  robot.printPath();
  robot.fastRun(true);
  robot.printPath();
#endif

#if 0
  const int n = 100;
  const bool diag_enabled = 1;
  const bool known_only = 0;
  std::chrono::microseconds sum{0};
  Maze maze = loadMaze();
  // Maze maze(loadMaze().getGoals());
  ShortestAlgorithm sa(maze);
  ShortestAlgorithm::Indexes path;
  for (int i = 0; i < n; ++i) {
    const auto t_s = std::chrono::system_clock().now();
    sa.calcShortestPath(path, known_only, diag_enabled);
    const auto t_e = std::chrono::system_clock().now();
    const auto us =
        std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
    sum += us;
  }
  sa.printPath(std::cout, path);
  std::cout << "It took " << sum.count() / n << " [us]" << std::endl;
#endif

#if 0
  Maze maze = loadMaze();
  std::cout << "uint8_t mazedata = {" << std::endl;
  for (int8_t x = 0; x < MAZE_SIZE; ++x) {
    std::cout << "{";
    for (int8_t y = 0; y < MAZE_SIZE; ++y) {
      int w = 0;
      if (maze.isWall(x, y, Dir::North))
        w |= 0x01;
      if (maze.isWall(x, y, Dir::East))
        w |= 0x02;
      if (maze.isWall(x, y, Dir::West))
        w |= 0x04;
      if (maze.isWall(x, y, Dir::South))
        w |= 0x08;
      std::cout << w << ",";
    }
    // std::cout << "}," << std::endl;
    std::cout << "},";
  }
  std::cout << "};" << std::endl;
#endif

#if 0
  const int n = 1;
  const bool diag_enabled = 1;
  const bool known_only = 0;
  std::chrono::microseconds sum{0};
  Maze maze = loadMaze();
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
