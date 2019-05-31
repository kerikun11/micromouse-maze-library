#include "Maze.h"
#include "RobotBase.h"
#include <chrono>
#include <cstdio>

using namespace MazeLib;

static Maze maze_target;
static bool display = 0;
static Vector offset_v;
static Dir offset_d;
static Vector real_v;
static Dir real_d;

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
    std::cout << "Real:\t" << VecDir{real_v, real_d} << std::endl;
    std::cout << "Offset:\t" << VecDir{offset_v, offset_d} << std::endl;
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
    if (newState == prevState)
      return;

    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      printInfo();
      // getc(stdin);
      display = 0;
    }
    if (newState == SearchAlgorithm::SEARCHING_ADDITIONALLY) {
    }
    if (newState == SearchAlgorithm::BACKING_TO_START) {
    }
    if (newState == SearchAlgorithm::REACHED_START) {
    }
  }
  void discrepancyWithKnownWall() override {
    printInfo();
    std::cout << "There was a discrepancy with known information! cur:\t"
              << VecDir{getCurVec(), getCurDir()} << std::endl;
  }
  void crashed() {
    printInfo();
    std::cerr << "The robot crashed into the wall! cur:\t"
              << VecDir{getCurVec(), getCurDir()} << std::endl;
    getc(stdin);
  }
  void queueAction(const Action action) override {
    if (display)
      printInfo();
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

static const Maze loadMaze() {
  switch (MAZE_SIZE) {
  case 8:
    return Maze("../mazedata/08MM2016CF_pre.maze");
  case 16:
    return Maze("../mazedata/16MM2016CX.maze");
  case 32:
    return Maze("../mazedata/32MM2017HX.maze");
  }
}

int main(void) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  /* Preparation */
  CLRobot robot;
  maze_target = loadMaze();
  robot.replaceGoals(maze_target.getGoals());

  /* Search Run */
  display = 0;
  robot.searchRun();

  /* Position Identification Run */
#if 0
  display = 1;
  offset_d = real_d = Dir::East;
  offset_v = real_v = Vector(31, 18);
  bool res = robot.positionIdentifyRun();
  if (!res) {
    robot.printInfo();
    std::cout << std::endl
              << "Failed to Identify! offset:\t" << VecDir{offset_v, offset_d}
              << std::endl;
    getc(stdin);
  }
#endif

  /* Starts from each cell on the shortest path */
  display = 1;
  for (auto diag : {true, false}) {
    robot.calcShortestDirs(diag);
    auto sdirs = robot.getShortestDirs();
    auto v = Vector(0, 0);
    for (const auto d : sdirs) {
      v = v.next(d);
      real_v = offset_v = v;
      for (const auto ed : Dir::ENWS()) {
        real_d = offset_d = ed;
        display = 1;
        bool res = robot.positionIdentifyRun();
        if (!res) {
          bool prev_display = display;
          display = 1;
          robot.printInfo();
          std::cout << std::endl
                    << "Failed to Identify! offset:\t"
                    << VecDir{offset_v, offset_d} << std::endl;
          getc(stdin);
          display = prev_display;
        }
      }
    }
  }
  return 0;
}
