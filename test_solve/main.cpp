#include "Maze.h"
#include "RobotBase.h"
#include <cstdio>

#include <chrono>
#include <time.h>
#include <unistd.h>

using namespace MazeLib;

Maze sample;

bool display = 0;

class TestRobot : public RobotBase {
public:
  TestRobot() : RobotBase(maze) {}

  void printInfo(const bool showMaze = true) {
    Agent::printInfo(showMaze);
    printf("Estimated Time: %2d:%02d, Step: %4d, Forward: %3d, Left: %3d, "
           "Right: %3d, Back: %3d\n",
           ((int)cost / 60) % 60, ((int)cost) % 60, step, f, l, r, b);
    printf("It took %5d [us], the max is %5d [us]\n", (int)usec, (int)max_usec);
  }

private:
  Maze maze;
  int step = 0, f = 0, l = 0, r = 0, b = 0; /**< 探索の評価のためのカウンタ */
  float cost = 0;
  int max_usec = 0;
  int usec;
  std::chrono::_V2::system_clock::time_point start;
  std::chrono::_V2::system_clock::time_point end;

  void findWall(bool &left, bool &front, bool &right, bool &back) override {
    const auto &v = getCurVec();
    const auto &d = getCurDir();
    left = sample.isWall(v, d + Dir::Left);
    front = sample.isWall(v, d + Dir::Front);
    right = sample.isWall(v, d + Dir::Right);
    back = sample.isWall(v, d + Dir::Back);
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

TestRobot robot;

int main(int argc, char *argv[]) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  if (argc < 2) {
    std::cout << "Please specify a maze file!" << std::endl;
    std::cout << "usage: $ test_solve <mazefile.maze>" << std::endl;
    return -1;
  }
  const auto filename = argv[1];
  if (!sample.parse(filename)) {
    std::cout << "Failed to parse " << filename << " !" << std::endl;
    return -1;
  }
  std::cout << "Solving " << filename << " ..." << std::endl;
  robot.replaceGoals(sample.getGoals());
  robot.searchRun();
  robot.printInfo();
  robot.fastRun(false);
  robot.endFastRunBackingToStartRun();
  robot.printPath();
  robot.fastRun(true);
  robot.endFastRunBackingToStartRun();
  robot.printPath();
  return 0;
}
