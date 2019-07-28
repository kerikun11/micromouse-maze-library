/**
 * @file CLRobotBase.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 時間計測や可視化を含むコマンドラインテスト用のRobotBase
 * @version 0.1
 * @date 2019-06-09
 *
 * @copyright Copyright (c) 2019
 *
 */
#pragma once

#include "Maze.h"
#include "RobotBase.h"
#include <chrono>
#include <cstdio>

namespace MazeLib {

class CLRobotBase : public RobotBase {
public:
  CLRobotBase(const Maze &maze_target)
      : RobotBase(maze), maze_target(maze_target) {}

  void printInfo(bool showMaze = true) {
    RobotBase::printInfo(showMaze);
    const auto nextDirs = getNextDirs();
    std::string path;
    auto prev_d = curDir;
    for (int i = 0; i < (int)nextDirs.size(); ++i) {
      const auto next_d = nextDirs[i];
      switch (Dir(next_d - prev_d)) {
      case Dir::Front:
        path += RobotBase::FastAction::F_ST_FULL;
        break;
      case Dir::Left:
        path += RobotBase::FastAction::FLS90;
        break;
      case Dir::Right:
        path += RobotBase::FastAction::FRS90;
        break;
      case Dir::Back:
        path += RobotBase::FastAction::TURN_BACK;
        break;
      default:
        loge << std::endl;
        break;
      }
      prev_d = next_d;
    }
    std::cout << "NextDirsKnown:     \x1b[0K";
    std::cout << path << std::endl;
    std::cout << "NextDirsKnownFast: \x1b[0K";
    std::cout << RobotBase::pathConvertSearchToKnown(path) << std::endl;
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
    // std::cout << "Max List:\t"
    //           <<
    //           getSearchAlgorithm().getShortestAlgorithm().max_open_list_size
    //           << std::endl;
    // std::cout << "Max Iteration:\t"
    //           <<
    //           getSearchAlgorithm().getShortestAlgorithm().max_iteration_size
    //           << std::endl;
  }
  bool endFastRunBackingToStartRun() {
    /* エラー処理 */
    if (getShortestDirs().empty()) {
      logw << "ShortestDirs are empty!" << std::endl;
      return false;
    }
    /* real を最短後の位置に移す */
    Vector v = maze.getStart();
    for (const auto d : getShortestDirs())
      v = v.next(d);
    real = VecDir(v, getShortestDirs().back());
    /* 基底関数を呼ぶ */
    return RobotBase::endFastRunBackingToStartRun();
  }

public:
  int step = 0, f = 0, l = 0, r = 0, b = 0; /**< 探索の評価のためのカウンタ */
  float cost = 0;
  int max_usec = 0;
  int usec = 0;
  size_t max_id_wall = 0;
  size_t min_id_wall = MAZE_SIZE * MAZE_SIZE * 4;
  std::chrono::system_clock::time_point start;
  std::chrono::system_clock::time_point end;

public:
  const Maze &maze_target;
  VecDir fake_offset;
  VecDir real;

protected:
  Maze maze;

  virtual void findWall(bool &left, bool &front, bool &right,
                        bool &back) override {
    left = maze_target.isWall(real.first, real.second + Dir::Left);
    front = maze_target.isWall(real.first, real.second + Dir::Front);
    right = maze_target.isWall(real.first, real.second + Dir::Right);
    back = maze_target.isWall(real.first, real.second + Dir::Back);
#if 0
    /* 前1区画先の壁を読める場合 */
    if (!front)
      updateWall(curVec.next(curDir), curDir,
                 maze_target.isWall(real.first.next(real.second), real.second));
#endif
  }
  virtual void calcNextDirsPreCallback() override {
    start = std::chrono::system_clock::now();
  }
  virtual void calcNextDirsPostCallback(SearchAlgorithm::State prevState
                                        __attribute__((unused)),
                                        SearchAlgorithm::State newState
                                        __attribute__((unused))) override {
    end = std::chrono::system_clock::now();
    usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start)
               .count();
    max_usec = std::max(max_usec, usec);
    if (newState == prevState)
      return;
    /* State Change has occurred */
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      min_id_wall = std::min(
          min_id_wall, getSearchAlgorithm().getIdMaze().getWallLogs().size());
      max_id_wall = std::max(
          max_id_wall, getSearchAlgorithm().getIdMaze().getWallLogs().size());
    }
    if (newState == SearchAlgorithm::IDENTIFYING_POSITION) {
    }
    if (newState == SearchAlgorithm::SEARCHING_ADDITIONALLY) {
    }
    if (newState == SearchAlgorithm::BACKING_TO_START) {
    }
    if (newState == SearchAlgorithm::REACHED_START) {
    }
  }
  virtual void discrepancyWithKnownWall() override {
    if (getState() != SearchAlgorithm::IDENTIFYING_POSITION) {
      printInfo();
      std::cout
          << "There was a discrepancy with known information! CurVecDir:\t"
          << VecDir{getCurVec(), getCurDir()} << std::endl;
    }
  }
  virtual void crashed() {
    std::cerr << "The robot crashed into the wall! fake_offset:\t"
              << fake_offset << "\treal:\t" << real << std::endl;
  }
  virtual void queueAction(const Action action) override {
    cost += getTimeCost(action);
    step++;
    switch (action) {
    case RobotBase::START_STEP:
      real.first = Vector(0, 1);
      real.second = Dir::North;
      f++;
      break;
    case RobotBase::START_INIT:
      break;
    case RobotBase::ST_HALF_STOP:
      break;
    case RobotBase::TURN_L:
      real.second = real.second + Dir::Left;
      if (!maze_target.canGo(real.first, real.second))
        crashed();
      real.first = real.first.next(real.second);
      l++;
      break;
    case RobotBase::TURN_R:
      real.second = real.second + Dir::Right;
      if (!maze_target.canGo(real.first, real.second))
        crashed();
      real.first = real.first.next(real.second);
      r++;
      break;
    case RobotBase::ROTATE_180:
      real.second = real.second + Dir::Back;
      if (!maze_target.canGo(real.first, real.second))
        crashed();
      real.first = real.first.next(real.second);
      b++;
      break;
    case RobotBase::ST_FULL:
      if (!maze_target.canGo(real.first, real.second))
        crashed();
      real.first = real.first.next(real.second);
      f++;
      break;
    case RobotBase::ST_HALF:
      break;
    }
  }
  virtual float getTimeCost(const Action action) {
    const float velocity = 240.0f;
    const float segment = 90.0f;
    switch (action) {
    case RobotBase::START_STEP:
      return 1.0f;
    case RobotBase::START_INIT:
      return 1.0f;
    case RobotBase::ST_HALF_STOP:
      return segment / 2 / velocity;
    case RobotBase::TURN_L:
      return 71 / velocity;
    case RobotBase::TURN_R:
      return 71 / velocity;
    case RobotBase::ROTATE_180:
      return 2.0f;
    case RobotBase::ST_FULL:
      return segment / velocity;
    case RobotBase::ST_HALF:
      return segment / 2 / velocity;
    }
    return 0;
  }
};

} // namespace MazeLib
