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
    /* 既知区間斜めの変換 */
    const auto nextDirections = getNextDirections();
    std::string path;
    auto prev_d = getCurrentPose().d;
    for (int i = 0; i < (int)nextDirections.size(); ++i) {
      const auto next_d = nextDirections[i];
      switch (Direction(next_d - prev_d)) {
      case Direction::Front:
        path += RobotBase::Action::ST_FULL;
        break;
      case Direction::Left:
        path += RobotBase::Action::TURN_L;
        break;
      case Direction::Right:
        path += RobotBase::Action::TURN_R;
        break;
      case Direction::Back:
        path += RobotBase::Action::ST_HALF_STOP;
        path += RobotBase::Action::ROTATE_180;
        path += RobotBase::Action::ST_HALF;
        break;
      default:
        loge << std::endl;
        break;
      }
      prev_d = next_d;
    }
    /* 表示 */
    // std::cout << "NextDirectionsKnown:     \x1b[0K";
    // std::cout << path << std::endl;
    // std::cout << "NextDirectionsKnownFast: \x1b[0K";
    // std::cout << RobotBase::pathConvertSearchToKnown(path, true) <<
    // std::endl;
    std::printf("Estimated Time: %2d:%02d, Step: %4d, Forward: %3d, Left: %3d, "
                "Right: %3d, Back: %3d\n",
                ((int)cost / 60) % 60, ((int)cost) % 60, step, f, l, r, b);
    // std::printf("It took %5d [us], the max is %5d [us]\n", t_dur, t_dur_max);
  }
  void printResult() const {
    std::printf("Estimated Time: %2d:%02d, Step: %4d, Forward: %3d, "
                "Left: %3d, Right: %3d, Back: %3d\n",
                ((int)cost / 60) % 60, ((int)cost) % 60, step, f, l, r, b);
    std::cout << "Walls:    \t" << maze.getWallLogs().size() << std::endl;
  }
  bool endFastRunBackingToStartRun() {
    /* エラー処理 */
    if (getShortestDirections().empty()) {
      logw << "ShortestDirections are empty!" << std::endl;
      return false;
    }
    /* real を最短後の位置に移す */
    Position p = maze.getStart();
    for (const auto d : getShortestDirections())
      p = p.next(d);
    real = Pose(p, getShortestDirections().back());
    /* 基底関数を呼ぶ */
    return RobotBase::endFastRunBackingToStartRun();
  }
  bool positionIdentifyRun() {
    step = f = l = r = b = cost = 0;
    return RobotBase::positionIdentifyRun();
  }
  void setMaze(const Maze &new_maze) { maze = new_maze; }

public:
  int step = 0, f = 0, l = 0, r = 0, b = 0; /**< 探索の評価のためのカウンタ */
  float cost = 0;                           /*< 探索時間 [秒] */
  size_t max_id_wall = 0;
  size_t min_id_wall = MAZE_SIZE * MAZE_SIZE * 4;
  int t_s;
  int t_e;
  int t_dur = 0;
  int t_dur_max = 0;

public:
  const Maze &maze_target;
  Pose fake_offset;
  Pose real;

protected:
  Maze maze;

  virtual void senseWalls(bool &left, bool &front, bool &right) override {
    left = !maze_target.canGo(real.p, real.d + Direction::Left);
    front = !maze_target.canGo(real.p, real.d + Direction::Front);
    right = !maze_target.canGo(real.p, real.d + Direction::Right);
#if 0
    /* 前1区画先の壁を読める場合 */
    if (!front)
      updateWall(current_pose.p.next(current_pose.d), current_pose.d,
                 !maze_target.canGo(real.p.next(real.d), real.d));
#endif
  }
  virtual void calcNextDirectionsPreCallback() override {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    t_s = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
  }
  virtual void calcNextDirectionsPostCallback(
      SearchAlgorithm::State prevState __attribute__((unused)),
      SearchAlgorithm::State newState __attribute__((unused))) override {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    t_e = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    t_dur = t_e - t_s;
    t_dur_max = std::max(t_dur_max, t_dur);
    if (newState == prevState)
      return;
    /* State Change has occurred */
    if (prevState == SearchAlgorithm::IDENTIFYING_POSITION) {
      min_id_wall = std::min(
          min_id_wall, getSearchAlgorithm().getIdMaze().getWallLogs().size());
      max_id_wall = std::max(
          max_id_wall, getSearchAlgorithm().getIdMaze().getWallLogs().size());
    }
  }
  virtual void discrepancyWithKnownWall() override {
    if (getState() != SearchAlgorithm::IDENTIFYING_POSITION) {
      logw << "There was a discrepancy with known information! "
           << getCurrentPose() << std::endl;
    }
  }
  virtual void backupMazeToFlash() override {
    maze.backupWallLogsFromFile("maze.walllogs");
  }
  virtual void queueAction(const Action action) override {
    cost += getTimeCost(action);
    step++;
    switch (action) {
    case RobotBase::START_STEP:
      real.p = Position(0, 1);
      real.d = Direction::North;
      f++;
      break;
    case RobotBase::START_INIT:
      break;
    case RobotBase::ST_HALF_STOP:
      break;
    case RobotBase::TURN_L:
      real.d = real.d + Direction::Left;
      if (!maze_target.canGo(real.p, real.d))
        crashed();
      real.p = real.p.next(real.d);
      l++;
      break;
    case RobotBase::TURN_R:
      real.d = real.d + Direction::Right;
      if (!maze_target.canGo(real.p, real.d))
        crashed();
      real.p = real.p.next(real.d);
      r++;
      break;
    case RobotBase::ROTATE_180:
      real.d = real.d + Direction::Back;
      if (!maze_target.canGo(real.p, real.d))
        crashed();
      real.p = real.p.next(real.d);
      b++;
      break;
    case RobotBase::ST_FULL:
      if (!maze_target.canGo(real.p, real.d))
        crashed();
      real.p = real.p.next(real.d);
      f++;
      break;
    case RobotBase::ST_HALF:
      break;
    default:
      logw << "invalid action" << std::endl;
      break;
    }
  }
  virtual void crashed() {
    loge << "The robot crashed into the wall! fake_offset:\t" << fake_offset
         << "\tcur:\t" << current_pose << "\treal:\t" << real << std::endl;
    setBreakFlag();
  }
  virtual float getTimeCost(const Action action) {
    const float velocity = 240.0f;
    const float segment = 90.0f;
    switch (action) {
    case RobotBase::START_STEP:
      return 1.0f;
    case RobotBase::START_INIT:
      return 3.0f;
    case RobotBase::ST_HALF_STOP:
      return segment / 2 / velocity;
    case RobotBase::TURN_L:
      return 71 / velocity;
    case RobotBase::TURN_R:
      return 71 / velocity;
    case RobotBase::ROTATE_180:
      return 3.0f;
    case RobotBase::ST_FULL:
      return segment / velocity;
    case RobotBase::ST_HALF:
      return segment / 2 / velocity;
    default:
      logw << "invalid action" << std::endl;
      return 0;
    }
  }
};

} // namespace MazeLib
