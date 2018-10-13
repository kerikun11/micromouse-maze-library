#include "Maze.h"
#include "RobotBase.h"
#include <cstdio>

#include <chrono>
#include <time.h>
#include <unistd.h>

using namespace MazeLib;

#define CONFIG_SERCHALGORITHM 1
#define CONFIG_POSITION_IDENTIFICATION 0

Maze sample;

void loadMaze() {
  switch (MAZE_SIZE) {
  case 8:
    sample.parse("../mazedata/08Test1.maze");
    // Maze sample("../mazedata/08MM2016CF_pre.maze");
    break;
  case 16:
    // Maze sample("../mazedata/16MM2017CX.maze");
    break;
  case 32:
    sample.parse("../mazedata/32MM2016HX.maze");
    // sample.parse("../mazedata/32MM2017CX.maze");
    break;
  }
}

#if CONFIG_SERCHALGORITHM

bool display = 0;
Vector offset;
Dir offset_d;

class TestRobot : public RobotBase {
public:
  TestRobot(const Vectors &goal) : RobotBase(goal) {}
  TestRobot() {}

  void printInfo(const bool showMaze = true) {
    Agent::printInfo(showMaze);
    printf("Estimated Time: %2d:%02d, Step: %4d, Forward: %3d, Left: %3d, "
           "Right: %3d, Back: %3d\n",
           ((int)cost / 60) % 60, ((int)cost) % 60, step, f, l, r, b);
    printf("It took %5d [us], the max is %5d [us]\n", (int)usec, (int)max_usec);
    printf("offset: ( %2d, %2d, %c)\n", offset.x, offset.y, ">^<v"[offset_d]);
    usleep(25000);
    // char c; scanf("%c", &c);
  }

private:
  int step = 0, f = 0, l = 0, r = 0, b = 0; /**< 探索の評価のためのカウンタ */
  float cost = 0;
  int max_usec = 0;
  int usec;
  std::chrono::_V2::system_clock::time_point start;
  std::chrono::_V2::system_clock::time_point end;

  void findWall(bool &left, bool &front, bool &right, bool &back) override {
    const auto &v = getCurVec();
    const auto &d = getCurDir();
    if (getState() == SearchAlgorithm::IDENTIFYING_POSITION) {
      auto fake_v = v + offset;
      auto fake_d = d;
      left = sample.isWall(fake_v, fake_d + Dir::Left);
      front = sample.isWall(fake_v, fake_d + Dir::Front);
      right = sample.isWall(fake_v, fake_d + Dir::Right);
      back = sample.isWall(fake_v, fake_d + Dir::Back);
    } else {
      left = sample.isWall(v, d + Dir::Left);
      front = sample.isWall(v, d + Dir::Front);
      right = sample.isWall(v, d + Dir::Right);
      back = sample.isWall(v, d + Dir::Back);
    }
  }
  void calcNextDirsPreCallback() override {
    start = std::chrono::system_clock::now();
  }
  void calcNextDirsPostCallback(SearchAlgorithm::State prevState,
                                SearchAlgorithm::State newState) override {
    end = std::chrono::system_clock::now();
    usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start)
               .count();
    if (max_usec < usec)
      max_usec = usec;
    if (newState == prevState)
      return;

    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
    }
    if (newState == SearchAlgorithm::SEARCHING_ADDITIONALLY) {
    }
    if (newState == SearchAlgorithm::BACKING_TO_START) {
    }
    if (newState == SearchAlgorithm::REACHED_START) {
#if CONFIG_POSITION_IDENTIFICATION
      printInfo();
      sleep(2);
#endif
    }
  }
  void discrepancyWithKnownWall() override {
    printInfo();
    printf("There was a discrepancy with known information!\n");
    // getc(stdin);
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

TestRobot robot;

#endif

int main(void) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
#if CONFIG_SERCHALGORITHM
  loadMaze();
  robot.replaceGoals(sample.getGoals());
  //   display = 1;
  robot.searchRun();
  robot.printInfo();
#if CONFIG_POSITION_IDENTIFICATION
  for (auto d : Dir::All()) {
    for (int x = -MAZE_SIZE / 2; x < 0 * MAZE_SIZE / 2; ++x)
      for (int y = -MAZE_SIZE / 2; y < 0 * MAZE_SIZE / 2; ++y) {
        offset = Vector(x, y);
        offset_d = d;
        bool res = robot.positionIdentifyRun(Dir::West);
        if (!res) {
          robot.printInfo();
          robot.positionIdentifyRun(Dir::West);
          printf("Failed to Identify! (%3d, %3d)\n", x, y);
          usleep(1000000);
          getc(stdin);
        }
      }
  }
#endif
  robot.fastRun(true);
  robot.endFastRunBackingToStartRun();
  robot.printPath();
  robot.fastRun(false);
  robot.endFastRunBackingToStartRun();
  robot.printPath();
#else
  std::ifstream ifs("MM2016HX_ip.maze");
  Maze m(ifs);
  Maze m("MM2016HX_ip.maze");
  m.parse(ifs);
  m.print(std::cout);

  sample.setGoals(goal);
  std::ofstream ofs(filename);
  sample.print(ofs);
  sample.print();
#endif
  return 0;
}
