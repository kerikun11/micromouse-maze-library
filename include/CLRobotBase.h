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
  CLRobotBase(Maze &maze_target) : RobotBase(maze), maze_target(maze_target) {}

  void printDoubleMaze(std::array<const Maze *, 2> maze,
                       std::array<const Pose *, 2> pose,
                       std::array<const StepMap *, 2> step_map,
                       std::ostream &os) const {
    /* preparation */
    const int maze_size = MAZE_SIZE;
    bool simple[2];
    for (int i = 0; i < 2; ++i) {
      StepMap::step_t max_step = 0;
      for (const auto step : step_map[i]->getMap())
        if (step != StepMap::STEP_MAX)
          max_step = std::max(max_step, step);
      simple[i] = (max_step < 999);
    }
    /* start to draw maze */
    for (int8_t y = maze_size; y >= 0; --y) {
      if (y != maze_size) {
        for (int i = 0; i < 2; ++i) {
          for (uint8_t x = 0; x <= maze_size; ++x) {
            /* Vertical Wall */
            const auto w = maze[i]->isWall(x, y, Direction::West);
            const auto k = maze[i]->isKnown(x, y, Direction::West);
            const auto d = pose[i]->d;
            if (WallIndex(pose[i]->p.next(d + Direction::Back), d) ==
                WallIndex(Position(x, y), Direction::West))
              os << "\e[43m"
                 << "\e[34m" << d << C_NO;
            else
              os << (k ? (w ? "|" : " ") : (C_RE "." C_NO));
            /* Cell */
            if (x != maze_size) {
              if (i == 0) {
                if (Position(x, y) == maze[i]->getStart())
                  os << C_BL << " S " << C_NO;
                else if (std::find(maze[i]->getGoals().cbegin(),
                                   maze[i]->getGoals().cend(),
                                   Position(x, y)) !=
                         maze[i]->getGoals().cend())
                  os << C_BL << " G " << C_NO;
                else
                  os << "   ";
              } else if (step_map[i]->getStep(x, y) == StepMap::STEP_MAX)
                os << C_CY << "999" << C_NO;
              else if (step_map[i]->getStep(x, y) == 0)
                os << C_YE << std::setw(3) << step_map[i]->getStep(x, y)
                   << C_NO;
              else if (simple[i])
                os << C_CY << std::setw(3) << step_map[i]->getStep(x, y)
                   << C_NO;
              else
                os << C_CY << std::setw(3) << step_map[i]->getStep(x, y) / 100
                   << C_NO;
            }
          }
          os << "   ";
        }
        os << std::endl;
      }
      for (int i = 0; i < 2; ++i) {
        for (uint8_t x = 0; x < maze_size; ++x) {
          /* Pillar */
          os << "+";
          /* Horizontal Wall */
          const auto w = maze[i]->isWall(x, y, Direction::South);
          const auto k = maze[i]->isKnown(x, y, Direction::South);
          const auto d = pose[i]->d;
          if (WallIndex(pose[i]->p.next(d + Direction::Back), d) ==
              WallIndex(Position(x, y), Direction::South))
            os << " "
               << "\e[43m"
               << "\e[34m" << pose[i]->d << C_NO << " ";
          else
            os << (k ? (w ? "---" : "   ") : (C_RE " . " C_NO));
        }
        /* Last Pillar */
        os << "+";
        os << "   ";
      }
      os << std::endl;
    }
  }
  void printInfo(const bool show_maze = true) {
#if 0
    if (show_maze) {
      std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
      printDoubleMaze(
          {&maze_target, state == SearchAlgorithm::IDENTIFYING_POSITION
                             ? &getSearchAlgorithm().getIdMaze()
                             : &maze},
          {&real, &current_pose},
          {&getSearchAlgorithm().getStepMap(),
           &getSearchAlgorithm().getStepMap()},
          std::cout);
    }
    RobotBase::printInfo(false);
    // std::cout << "Real Pose:\t" << real << std::endl;
#else
    RobotBase::printInfo(show_maze);
#endif
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
  bool fastRun(const bool diag_enabled) {
    /* 最短経路の導出 */
    if (!calcShortestDirections(diag_enabled)) {
      loge << "Failed to find shortest path!" << std::endl;
      return false;
    }
    /* 現在位置をスタートに設定 */
    const auto pose = Pose(maze.getStart(), getShortestDirections()[0]);
    updateCurrentPose(pose);
    real = pose;
    /* 移動 */
    queueNextDirections(getShortestDirections());
    return true;
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
  bool positionIdentifyRun(const bool reset_step = true) {
    if (reset_step)
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
  Maze &maze_target;
  Pose fake_offset;
  Pose real;
  bool real_visit_goal = false;

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
    // maze.backupWallLogsToFile("maze.walllogs");
  }
  virtual void queueAction(const Action action) override {
    const auto goals = maze.getGoals();
    if (std::find(goals.cbegin(), goals.cend(), current_pose.p) != goals.cend())
      real_visit_goal = true;
    cost += getTimeCost(action);
    step++;
    switch (action) {
    case RobotBase::START_STEP:
      real.p = Position(0, 1);
      real.d = Direction::North;
      real_visit_goal = false;
      f++;
      break;
    case RobotBase::START_INIT:
      if (real_visit_goal == false)
        loge << "Reached Start without Going to Goal!" << std::endl;
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
    const float velocity = 300.0f;
    const float segment = 90.0f;
    switch (action) {
    case RobotBase::START_STEP:
      return 3.0f;
    case RobotBase::START_INIT:
      return 3.0f;
    case RobotBase::ST_HALF_STOP:
      return segment / 2 / velocity;
    case RobotBase::TURN_L:
      return 71 / velocity;
    case RobotBase::TURN_R:
      return 71 / velocity;
    case RobotBase::ROTATE_180:
      return 5.0f;
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
