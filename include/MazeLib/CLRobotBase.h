/**
 * @file CLRobotBase.h
 * @brief 時間計測や可視化を含むコマンドラインテスト用のロボット基底
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-06-09
 * @copyright Copyright 2019 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include "MazeLib/RobotBase.h"

#include <algorithm> /*< for std::find */
#include <chrono>
#include <cstdio>  /*< for std::printf */
#include <iomanip> /*< for std::setw */

namespace MazeLib {

/**
 * @brief 時間計測や可視化を含むコマンドラインテスト用の RobotBase
 */
class CLRobotBase : public RobotBase {
 public:
  CLRobotBase(Maze& mazeTarget) : mazeTarget(mazeTarget) {
    replaceGoals(mazeTarget.getGoals());
  }
  void printInfo(const bool showMaze = true) {
    RobotBase::printInfo(showMaze);
    printSearchResult();
  }
  void printInfoDoubleMaze(const bool showMaze = true) {
    if (showMaze) {
      std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
      printDoubleMaze(
          {&getMaze(), getState() == SearchAlgorithm::IDENTIFYING_POSITION
                           ? &getSearchAlgorithm().getIdMaze()
                           : &maze},
          {&real, &getCurrentPose()},
          {&getSearchAlgorithm().getStepMap(),
           &getSearchAlgorithm().getStepMap()},
          std::cout);
    }
    RobotBase::printInfo(false);
    std::cout << "Real Pose:\t" << real << std::endl;
  }
  void printSearchResult() const {
    std::printf(
        "SearchTime: %2d:%02d, Step: %4d, "
        "F: %4d, L: %3d, R: %3d, B: %3d, Walls: %4d\n",
        est_time / 1000 / 60, est_time / 1000 % 60, step, f, l, r, b,
        int(maze.getWallRecords().size()));
  }
  void printFastResult(const bool diagEnabled, const bool showMaze = false) {
    /* 最短走行時間の表示 */
    const auto path_cost = getSearchAlgorithm().getShortestCost();
    std::cout << "PathCost " << (diagEnabled ? "diag" : "no_d") << ":\t"
              << path_cost << "\t[ms]" << std::endl;
    if (showMaze) {
      printPath();
    }
    /* 最短経路の比較 */
    const auto pAgent = std::make_unique<Agent>();
    Agent& at = *pAgent;
    at.updateMaze(mazeTarget);
    at.calcShortestDirections(diagEnabled);
    calcShortestDirections(diagEnabled);
    if (at.getSearchAlgorithm().getShortestCost() !=
        getSearchAlgorithm().getShortestCost()) {
      MAZE_LOGW << "searched path is not shortest! "
                << (diagEnabled ? "(diag)" : "(no_diag)") << std::endl;
      MAZE_LOGW << "real: " << at.getSearchAlgorithm().getShortestCost()
                << " searched: " << getSearchAlgorithm().getShortestCost()
                << std::endl;
      // at.printPath(), robot.printPath();
    }
  }
  void printSearchLogs(std::ostream& os) {
    for (const auto& data : calcNextDirectionsData) {
      os << SearchAlgorithm::getStateString(data.state) << "\t";
      os << data.tDur << "\n";
    }
  }
  bool searchRun() {
    if (!RobotBase::searchRun()) {
      MAZE_LOGE << "searchRun failed." << std::endl;
      return false;
    }
    return true;
  }
  bool fastRun(const bool diagEnabled) {
    /* 最短経路の導出 */
    if (!calcShortestDirections(diagEnabled)) {
      MAZE_LOGW << "Failed to find shortest path!" << std::endl;
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
      MAZE_LOGE << "endFastRunBackingToStartRun failed." << std::endl;
      return false;
    }
    return true;
  }
  bool positionIdentifyRun(const bool reset_cost = true) {
    if (reset_cost) {
      step = f = l = r = b = est_time = 0;
      calcNextDirectionsData.clear();
    }
    if (!RobotBase::positionIdentifyRun()) {
      MAZE_LOGE << "positionIdentifyRun failed." << std::endl;
      return false;
    }
    return true;
  }

 public:
  int step = 0, f = 0, l = 0, r = 0, b = 0; /*< 探索の評価のためのカウンタ */
  uint32_t est_time = 0;                    /*< 探索時間 [ms] */

 public:
  size_t walls_pi_max = 0;
  size_t walls_pi_min = MAZE_SIZE * MAZE_SIZE * 4;
  uint32_t pi_est_time_max = 0;
  uint32_t pi_est_time_min = std::numeric_limits<uint32_t>::max();

 public:
  int tCalcNextDirsPrev;
  int tCalcMax = 0;
  struct CalcNextDirectionsData {
    SearchAlgorithm::State state;
    Pose currentPose;
    int tDur;
  };
  std::vector<CalcNextDirectionsData> calcNextDirectionsData;

 public:
  Maze& mazeTarget;
  Pose fake_offset;
  Pose real;
  bool real_visit_goal = false;

 protected:
  SearchAction action_prev = SearchAction::START_STEP;
  bool unknown_accel_prev = false;

  virtual void senseWalls(bool& left, bool& front, bool& right) override {
    left = !mazeTarget.canGo(real.p, real.d + Direction::Left);
    front = !mazeTarget.canGo(real.p, real.d + Direction::Front);
    right = !mazeTarget.canGo(real.p, real.d + Direction::Right);
#if 0
    /* 前1区画先の壁を読める場合 */
    if (!front)
      updateWall(currentPose.p.next(currentPose.d), currentPose.d,
                 !mazeTarget.canGo(real.p.next(real.d), real.d));
#endif
  }
  virtual void calcNextDirectionsPreCallback() override {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    tCalcNextDirsPrev =
        std::chrono::duration_cast<std::chrono::microseconds>(now).count();
  }
  virtual void calcNextDirectionsPostCallback(
      SearchAlgorithm::State oldState,
      SearchAlgorithm::State newState) override {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const int tCalcNextDirsPost =
        std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    const int tCalc = tCalcNextDirsPost - tCalcNextDirsPrev;
    tCalcMax = std::max(tCalcMax, tCalc);
    calcNextDirectionsData.push_back({getState(), getCurrentPose(), tCalc});
    if (newState == oldState)
      return;
    /* State Change has occurred */
    if (oldState == SearchAlgorithm::IDENTIFYING_POSITION) {
      const auto walls =
          getSearchAlgorithm().getIdMaze().getWallRecords().size();
      walls_pi_min = std::min(walls_pi_min, walls);
      walls_pi_max = std::max(walls_pi_max, walls);
      pi_est_time_max = std::max(pi_est_time_max, est_time);
      pi_est_time_min = std::min(pi_est_time_min, est_time);
    }
  }
  virtual void discrepancyWithKnownWall() override {
    if (getState() != SearchAlgorithm::IDENTIFYING_POSITION) {
      MAZE_LOGW << "There was a discrepancy with known information! "
                << getCurrentPose() << std::endl;
    }
  }
  virtual void backupMazeToFlash() override {
    // maze.backupWallRecordsToFile("maze.wallRecords"); //< (takes some time)
  }
  virtual void queueAction(const SearchAction action) override {
#if 1
    /* 自己位置同定走行中のスタート区画訪問の警告 */
    if (getState() == SearchAlgorithm::IDENTIFYING_POSITION &&
        real.p == maze.getStart() && action != ST_HALF_STOP &&
        !(fake_offset.p.x == 0 && fake_offset.d == Direction::South &&
          mazeTarget.isWall(fake_offset.p, Direction::East)))
      MAZE_LOGW << "Visited Start at P.I. fake_offset: " << fake_offset
                << std::endl;
#endif
#if 1
    /* 未知区間加速のバグ探し */
    if (unknown_accel_prev && action_prev == SearchAction::ST_FULL &&
        action != SearchAction::ST_FULL &&
        getNextDirectionsKnown().size() == 0 &&
        !maze.isWall(getCurrentPose().p, getCurrentPose().d)) {
      printInfo();
      MAZE_LOGW << "not straight in unknown accel" << std::endl;
      getc(stdin);
    }
    unknown_accel_prev = getState() != SearchAlgorithm::GOING_TO_GOAL &&
                         getState() != SearchAlgorithm::IDENTIFYING_POSITION &&
                         getUnknownAccelFlag();
#endif
    const auto goals = maze.getGoals();
    if (std::find(goals.cbegin(), goals.cend(), getCurrentPose().p) !=
        goals.cend())
      real_visit_goal = true;
    est_time += getTimeCost(action);
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
          MAZE_LOGW << "Reached Start without Going to Goal!" << std::endl;
        break;
      case RobotBase::ST_HALF_STOP:
        break;
      case RobotBase::TURN_L:
        real.d = real.d + Direction::Left;
        if (!mazeTarget.canGo(real.p, real.d))
          crashed();
        real.p = real.p.next(real.d);
        l++;
        step++;
        break;
      case RobotBase::TURN_R:
        real.d = real.d + Direction::Right;
        if (!mazeTarget.canGo(real.p, real.d))
          crashed();
        real.p = real.p.next(real.d);
        r++;
        step++;
        break;
      case RobotBase::ROTATE_180:
        real.d = real.d + Direction::Back;
        if (!mazeTarget.canGo(real.p, real.d))
          crashed();
        real.p = real.p.next(real.d);
        b++;
        step++;
        break;
      case RobotBase::ST_FULL:
        if (!mazeTarget.canGo(real.p, real.d))
          crashed();
        real.p = real.p.next(real.d);
        /* 未知区間加速 */
        if (getUnknownAccelFlag() && action_prev == action)
          est_time -= getTimeCost(action) / 3;
        /* 既知区間加速 */
        if (getNextDirectionsKnown().size() > 1 && action_prev == action)
          est_time -= getTimeCost(action) / 2;
        f++;
        step++;
        break;
      case RobotBase::ST_HALF:
        break;
      default:
        MAZE_LOGE << "invalid action" << std::endl;
        break;
    }
    action_prev = action;
  }
  virtual void crashed() {
    printInfo();
    MAZE_LOGE << "The robot crashed into the wall! fake_offset:\t"
              << fake_offset << "\tcur:\t" << getCurrentPose() << "\treal:\t"
              << real << std::endl;
    getc(stdin);
    setBreakFlag();
  }
  virtual uint32_t getTimeCost(const SearchAction action) {
    const float velocity = 300.0f;
    const float segment = 90.0f;
    switch (action) {
      case RobotBase::START_STEP:
        return 3000;
      case RobotBase::START_INIT:
        return 3000;
      case RobotBase::ST_HALF_STOP:
        return segment / 2 / velocity * 1000;
      case RobotBase::TURN_L:
        return 71 / velocity * 1000;
      case RobotBase::TURN_R:
        return 71 / velocity * 1000;
      case RobotBase::ROTATE_180:
        return 3000;
      case RobotBase::ST_FULL:
        return segment / velocity * 1000;
      case RobotBase::ST_HALF:
        return segment / 2 / velocity * 1000;
      default:
        MAZE_LOGE << "invalid action" << std::endl;
        return 0;
    }
  }

 protected:
  void printDoubleMaze(std::array<const Maze*, 2> maze,
                       std::array<const Pose*, 2> pose,
                       std::array<const StepMap*, 2> stepMap,
                       std::ostream& os) const {
    /* preparation */
    const int mazeSize = MAZE_SIZE;
    bool simple[2];
    for (int i = 0; i < 2; ++i) {
      StepMap::step_t maxStep = 0;
      for (const auto step : stepMap[i]->getMapArray())
        if (step != StepMap::STEP_MAX)
          maxStep = std::max(maxStep, step);
      simple[i] = (maxStep < 999);
    }
    /* start to draw maze */
    for (int8_t y = mazeSize; y >= 0; --y) {
      if (y != mazeSize) {
        for (int i = 0; i < 2; ++i) {
          for (uint8_t x = 0; x <= mazeSize; ++x) {
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
            if (x != mazeSize) {
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
              } else if (stepMap[i]->getStep(x, y) == StepMap::STEP_MAX)
                os << C_CY << "999" << C_NO;
              else if (stepMap[i]->getStep(x, y) == 0)
                os << C_YE << std::setw(3) << stepMap[i]->getStep(x, y) << C_NO;
              else if (simple[i])
                os << C_CY << std::setw(3) << stepMap[i]->getStep(x, y) << C_NO;
              else
                os << C_CY << std::setw(3) << stepMap[i]->getStep(x, y) / 100
                   << C_NO;
            }
          }
          os << "   ";
        }
        os << std::endl;
      }
      for (int i = 0; i < 2; ++i) {
        for (uint8_t x = 0; x < mazeSize; ++x) {
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

}  // namespace MazeLib
