/**
 * @file CLRobotBase.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 時間計測や可視化を含むコマンドラインテスト用のロボット基底
 * @copyright Copyright (c) 2019 Ryotaro Onuki
 * @date 2019-06-09
 */
#pragma once

#include "MazeLib/RobotBase.h"
#include <chrono>
#include <cstdio>  /*< for std::printf */
#include <iomanip> /*< for std::setw */

namespace MazeLib {

/**
 * @brief 時間計測や可視化を含むコマンドラインテスト用の RobotBase
 */
class CLRobotBase : public RobotBase {
public:
  CLRobotBase(Maze &maze_target) : maze_target(maze_target) {
    replaceGoals(maze_target.getGoals());
  }
  void printInfo(const bool show_maze = true) {
    RobotBase::printInfo(show_maze);
    printSearchResult();
  }
  void printInfoDoubleMaze(const bool show_maze = true) {
    if (show_maze) {
      std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
      printDoubleMaze(
          {&maze_target, getState() == SearchAlgorithm::IDENTIFYING_POSITION
                             ? &getSearchAlgorithm().getIdMaze()
                             : &maze},
          {&real, &current_pose},
          {&getSearchAlgorithm().getStepMap(),
           &getSearchAlgorithm().getStepMap()},
          std::cout);
    }
    RobotBase::printInfo(false);
    std::cout << "Real Pose:\t" << real << std::endl;
  }
  void printSearchResult() const {
    std::printf("SearchTime: %2d:%02d, Step: %4d, "
                "F: %4d, L: %3d, R: %3d, B: %3d, Walls: %4d\n",
                int(cost / 60) % 60, int(cost) % 60, step, f, l, r, b,
                int(maze.getWallRecords().size()));
  }
  void printFastResult(const bool diag_enabled, const bool show_maze = false) {
    /* 最短走行時間の表示 */
    const auto path_cost = getSearchAlgorithm().getShortestCost();
    std::cout << "PathCost " << (diag_enabled ? "diag" : "no_d") << ":\t"
              << path_cost << "\t[ms]" << std::endl;
    if (show_maze) {
      printPath();
    }
    /* 最短経路の比較 */
    const auto p_at = std::make_unique<Agent>(maze_target);
    Agent &at = *p_at;
    at.calcShortestDirections(diag_enabled);
    calcShortestDirections(diag_enabled);
    if (at.getSearchAlgorithm().getShortestCost() !=
        getSearchAlgorithm().getShortestCost()) {
      maze_logw << "searched path is not shortest! "
                << (diag_enabled ? "(diag)" : "(no_diag)") << std::endl;
      maze_logw << "real: " << at.getSearchAlgorithm().getShortestCost()
                << " searched: " << getSearchAlgorithm().getShortestCost()
                << std::endl;
      // at.printPath(), robot.printPath();
    }
  }
  bool searchRun() {
    if (!RobotBase::searchRun()) {
      maze_loge << "searchRun failed." << std::endl;
      return false;
    }
    return true;
  }
  bool fastRun(const bool diag_enabled) {
    /* 最短経路の導出 */
    if (!calcShortestDirections(diag_enabled)) {
      maze_logw << "Failed to find shortest path!" << std::endl;
      return false;
    }
    /* 現在位置をスタート区画に設定 */
    const auto pose = Pose(maze.getStart(), getShortestDirections()[0]);
    updateCurrentPose(pose);
    real = pose;
    /* 最短走行後の位置に移動 */
    queueNextDirections(getShortestDirections());
    /* real を最短後の位置に移動 */
    Position p = maze.getStart();
    for (const auto d : getShortestDirections())
      p = p.next(d);
    real = Pose(p, getShortestDirections().back());
    /* スタート区画に帰る */
    if (!endFastRunBackingToStartRun()) {
      maze_loge << "endFastRunBackingToStartRun failed." << std::endl;
      return false;
    }
    return true;
  }
  bool positionIdentifyRun(const bool reset_cost = true) {
    if (reset_cost)
      step = f = l = r = b = cost = 0;
    if (!RobotBase::positionIdentifyRun()) {
      maze_loge << "positionIdentifyRun failed." << std::endl;
      return false;
    }
    return true;
  }

public:
  int step = 0, f = 0, l = 0, r = 0, b = 0; /*< 探索の評価のためのカウンタ */
  float cost = 0;                           /*< 探索時間 [秒] */
  size_t walls_pi_max = 0;
  size_t walls_pi_min = MAZE_SIZE * MAZE_SIZE * 4;
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
  SearchAction action_prev = SearchAction::START_STEP;
  bool unknown_accel_prev = false;

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
      const auto walls =
          getSearchAlgorithm().getIdMaze().getWallRecords().size();
      walls_pi_min = std::min(walls_pi_min, walls);
      walls_pi_max = std::max(walls_pi_max, walls);
    }
  }
  virtual void discrepancyWithKnownWall() override {
    if (getState() != SearchAlgorithm::IDENTIFYING_POSITION) {
      maze_logw << "There was a discrepancy with known information! "
                << getCurrentPose() << std::endl;
    }
  }
  virtual void backupMazeToFlash() override {
    // maze.backupWallRecordsToFile("maze.wallRecords"); //< (takes some time)
  }
  virtual void queueAction(const SearchAction action) override {
#if 1
    /* 未知区間加速のバグ探し */
    if (unknown_accel_prev && action_prev == SearchAction::ST_FULL &&
        action != SearchAction::ST_FULL && getNextDirections().size() == 0 &&
        !maze.isWall(current_pose.p, current_pose.d)) {
      printInfo();
      maze_logw << "not straight in unknown accel" << std::endl;
      getc(stdin);
    }
    unknown_accel_prev = getState() != SearchAlgorithm::GOING_TO_GOAL &&
                         getState() != SearchAlgorithm::IDENTIFYING_POSITION &&
                         getUnknownAccelFlag();
#endif
    const auto goals = maze.getGoals();
    if (std::find(goals.cbegin(), goals.cend(), current_pose.p) != goals.cend())
      real_visit_goal = true;
    cost += getTimeCost(action);
    switch (action) {
    case RobotBase::START_STEP:
      real.p = Position(0, 1);
      real.d = Direction::North;
      real_visit_goal = false;
      f++;
      step++;
      break;
    case RobotBase::START_INIT:
      if (real_visit_goal == false)
        maze_logw << "Reached Start without Going to Goal!" << std::endl;
      break;
    case RobotBase::ST_HALF_STOP:
      break;
    case RobotBase::TURN_L:
      real.d = real.d + Direction::Left;
      if (!maze_target.canGo(real.p, real.d))
        crashed();
      real.p = real.p.next(real.d);
      l++;
      step++;
      break;
    case RobotBase::TURN_R:
      real.d = real.d + Direction::Right;
      if (!maze_target.canGo(real.p, real.d))
        crashed();
      real.p = real.p.next(real.d);
      r++;
      step++;
      break;
    case RobotBase::ROTATE_180:
      real.d = real.d + Direction::Back;
      if (!maze_target.canGo(real.p, real.d))
        crashed();
      real.p = real.p.next(real.d);
      b++;
      step++;
      break;
    case RobotBase::ST_FULL:
      if (!maze_target.canGo(real.p, real.d))
        crashed();
      real.p = real.p.next(real.d);
      /* 未知区間加速 */
      if (getUnknownAccelFlag() && action_prev == action)
        cost -= getTimeCost(action) / 3;
      /* 既知区間加速 */
      if (getNextDirections().size() > 1 && action_prev == action)
        cost -= getTimeCost(action) / 2;
      f++;
      step++;
      break;
    case RobotBase::ST_HALF:
      break;
    default:
      maze_loge << "invalid action" << std::endl;
      break;
    }
    action_prev = action;
  }
  virtual void crashed() {
    maze_loge << "The robot crashed into the wall! fake_offset:\t"
              << fake_offset << "\tcur:\t" << current_pose << "\treal:\t"
              << real << std::endl;
    setBreakFlag();
  }
  virtual float getTimeCost(const SearchAction action) {
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
      return 3.0f;
    case RobotBase::ST_FULL:
      return segment / velocity;
    case RobotBase::ST_HALF:
      return segment / 2 / velocity;
    default:
      maze_loge << "invalid action" << std::endl;
      return 0;
    }
  }

protected:
  void printDoubleMaze(std::array<const Maze *, 2> maze,
                       std::array<const Pose *, 2> pose,
                       std::array<const StepMap *, 2> step_map,
                       std::ostream &os) const {
    /* preparation */
    const int maze_size = MAZE_SIZE;
    bool simple[2];
    for (int i = 0; i < 2; ++i) {
      StepMap::step_t max_step = 0;
      for (const auto step : step_map[i]->getMapArray())
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
};

} // namespace MazeLib
