#include "Maze.h"
#include "RobotBase.h"
#include <cstdio>

#include <chrono>
#include <time.h>
#include <unistd.h>

using namespace MazeLib;

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
    // sample.parse("../mazedata/32MM2016HX.maze");
    sample.parse("../mazedata/32MM2017CX.maze");
    break;
  }
}

bool display = 0;
Vector offset_v;
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
    printf("offset: (%3d,%3d,%3c)\n", offset_v.x, offset_v.y, ">^<v"[offset_d]);
    // usleep(25000);
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
      auto ids = Vector(MAZE_SIZE / 2, MAZE_SIZE / 2);
      auto fake_v = v.rotate(offset_d, ids) + offset_v;
      auto fake_d = d + offset_d;
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
      sleep(2);
      //   getc(stdin);
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
    printf("There was a discrepancy with known information!\n");
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

int main(void) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  loadMaze();
  TestRobot robot;
  robot.replaceGoals(sample.getGoals());
  // display = 1;
  robot.searchRun();
  robot.printInfo();
  sleep(2);
  // for (int x = -MAZE_SIZE / 2; x < MAZE_SIZE / 2; ++x)
  //   for (int y = -MAZE_SIZE / 2; y < MAZE_SIZE / 2; ++y)
  //     for (auto d : Dir::All()) {
  //       offset_d = d;
  //       offset_v = Vector(x, y);
  //       display = 1;
  //       bool res = robot.positionIdentifyRun();
  //       if (!res) {
  //         robot.printInfo();
  //         printf("Failed to Identify! (%3d, %3d)\n", x, y);
  //         usleep(1000000);
  //         getc(stdin);
  //       }
  //     }
  robot.calcShortestDirs();
  auto sdirs = robot.getShortestDirs();
  auto v = Vector(0, 0);
  for (const auto &d : sdirs) {
    v = v.next(d);
    offset_v = v - Vector(MAZE_SIZE / 2, MAZE_SIZE / 2);
    for (const auto ed : Dir::All()) {
      offset_d = ed;
      display = 1;
      bool res = robot.positionIdentifyRun();
      if (!res) {
        robot.printInfo();
        printf("\nFailed to Identify!\n");
        getc(stdin);
      }
    }
  }
  display = 0;
  robot.fastRun(false);
  robot.endFastRunBackingToStartRun();
  robot.printPath();
  robot.fastRun(true);
  robot.endFastRunBackingToStartRun();
  robot.printPath();
  return 0;
}