/**
 * @file SearchAlgorithm.cpp
 * @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2017-11-05
 * @copyright Copyright 2017 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include "MazeLib/SearchAlgorithm.h"

#include <algorithm> /*< for std::find */

namespace MazeLib {

/**
 * @brief 追加探索状態で探索を始める(ゴールを急がない)
 */
#define SEARCHING_ADDITIONALLY_AT_START 1

#define STEP_MAP_RECOVERY_DEBUG_MODE 0

const char* SearchAlgorithm::getStateString(const State s) {
  static const char* const str[] = {
      "Start                 ", "Searching for Goal    ",
      "Searching Additionally", "Backing to Start      ",
      "Reached Start         ", "Impossible            ",
      "Identifying Position  ", "Going to Goal         ",
  };
  return str[s];
}
bool SearchAlgorithm::isCompleted() {
  Positions candidates;
  if (!findShortestCandidates(candidates))
    return false;
  return candidates.empty();
}
bool SearchAlgorithm::isSolvable() {
  return !step_map.calcShortestDirections(maze, false, false).empty();
}
void SearchAlgorithm::positionIdentifyingInit(Pose& currentPose) {
  /* オフセットを迷路の中央に設定 */
  idOffset = Position(MAZE_SIZE / 2, MAZE_SIZE / 2);
  currentPose = Pose(idOffset, Direction::East);
  idMaze.reset(false); /*< reset without setting start cell */
  /* 自分の真下の壁を消す */
  idMaze.updateWall(currentPose.p, currentPose.d + Direction::Back, false);
}
bool SearchAlgorithm::updateWall(const State state,
                                 const Pose& pose,
                                 const bool left,
                                 const bool front,
                                 const bool right) {
  bool result = true;
  if (!updateWall(state, pose.p, pose.d + Direction::Left, left))
    result = false;
  if (!updateWall(state, pose.p, pose.d + Direction::Front, front))
    result = false;
  if (!updateWall(state, pose.p, pose.d + Direction::Right, right))
    result = false;
  return result;
}
bool SearchAlgorithm::updateWall(const State state,
                                 const Position p,
                                 const Direction d,
                                 const bool b) {
  auto& m = (state == IDENTIFYING_POSITION) ? idMaze : maze;
  return m.updateWall(p, d, b);
}
void SearchAlgorithm::resetLastWalls(const State state, const int num) {
  if (state == IDENTIFYING_POSITION) {
    return idMaze.resetLastWalls(num, false);
  }
  return maze.resetLastWalls(num, true);
}
void SearchAlgorithm::updatePose(const State& state,
                                 Pose& currentPose,
                                 bool& isForceGoingToGoal) {
  if (state == SearchAlgorithm::IDENTIFYING_POSITION)
    return;
  if (!isForceGoingToGoal)
    return;
  const auto& goals = maze.getGoals();
  if (std::find(goals.cbegin(), goals.cend(), currentPose.p) != goals.cend())
    isForceGoingToGoal = false;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirections(
    SearchAlgorithm::NextDirections& nextDirections,
    Pose& currentPose,
    bool& isPositionIdentifying,
    bool& isForceBackToStart,
    bool& isForceGoingToGoal) {
  /* initialize */
  nextDirections.state = State::START;
  nextDirections.unknownAccelFlag = false;
  /* position identification */
  if (isPositionIdentifying) {
    nextDirections.state = State::IDENTIFYING_POSITION;
    Result result = calcNextDirectionsPositionIdentification(
        nextDirections, currentPose, isForceGoingToGoal);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        isPositionIdentifying = false;
        break; /*< go to next state */
      case SearchAlgorithm::Error:
        nextDirections.state = State::IMPOSSIBLE;
        return result;
      default:
        maze_loge << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* search for goal */
  if (!SEARCHING_ADDITIONALLY_AT_START) {
    nextDirections.state = State::SEARCHING_FOR_GOAL;
    Result result =
        calcNextDirectionsSearchForGoal(nextDirections, currentPose);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        break; /*< go to next state */
      case SearchAlgorithm::Error:
        nextDirections.state = State::IMPOSSIBLE;
        return result;
      default:
        maze_loge << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* searching additionally */
  if (!isForceBackToStart) {
    nextDirections.state = State::SEARCHING_ADDITIONALLY;
    Result result =
        calcNextDirectionsSearchAdditionally(nextDirections, currentPose);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        break; /*< go to next state */
      case SearchAlgorithm::Error:
        nextDirections.state = State::IMPOSSIBLE;
        return result;
      default:
        maze_loge << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* force going to goal */
  if (isForceGoingToGoal) {
    nextDirections.state = State::GOING_TO_GOAL;
    Result result = calcNextDirectionsGoingToGoal(nextDirections, currentPose);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        isForceGoingToGoal = false;
        break; /*< go to next state */
      case SearchAlgorithm::Error:
        nextDirections.state = State::IMPOSSIBLE;
        return result;
      default:
        maze_loge << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* backing to start */
  nextDirections.state = State::BACKING_TO_START;
  Result result = calcNextDirectionsBackingToStart(nextDirections, currentPose);
  switch (result) {
    case SearchAlgorithm::Processing:
      return result;
    case SearchAlgorithm::Reached:
      isForceBackToStart = false;
      break; /*< go to next state */
    case SearchAlgorithm::Error:
      nextDirections.state = State::IMPOSSIBLE;
      return result;
    default:
      maze_loge << "invalid result" << std::endl;
      return SearchAlgorithm::Error;
  }
  /* reached start */
  nextDirections.state = State::REACHED_START;
  return result;
}
bool SearchAlgorithm::determineNextDirection(
    const State state,
    const Pose& pose,
    const Directions& nextDirectionCandidates,
    Direction& nextDirection) const {
  const auto& m = (state == State::IDENTIFYING_POSITION) ? idMaze : maze;
  return determineNextDirection(m, pose, nextDirectionCandidates,
                                nextDirection);
}
bool SearchAlgorithm::determineNextDirection(
    const Maze& maze,
    const Pose& pose,
    const Directions& nextDirectionCandidates,
    Direction& nextDirection) const {
  /* find a direction it can go in nextDirectionCandidates */
  const auto it = std::find_if(
      nextDirectionCandidates.cbegin(), nextDirectionCandidates.cend(),
      [&](const Direction next_d) { return maze.canGo(pose.p, next_d); });
  if (it == nextDirectionCandidates.cend())
    return false; /*< no answer */
  nextDirection = *it;
  return true;
}
bool SearchAlgorithm::calcShortestDirections(
    Directions& shortestDirections,
    const bool diagEnabled,
    const StepMapSlalom::EdgeCost& edgeCost) {
  const bool knownOnly = true;
  if (diagEnabled) {
    shortestDirections =
        step_map_slalom.calcShortestDirections(maze, edgeCost, knownOnly);
    if (shortestDirections.empty())
      return false; /*< failed */
    cost = step_map_slalom.getShortestCost();
  } else {
    shortestDirections =
        step_map.calcShortestDirections(maze, knownOnly, false);
    if (shortestDirections.empty())
      return false; /* no path to goal */
    cost = step_map.getStep(maze.getStart()) * step_map.getScalingFactor();
  }
  StepMap::appendStraightDirections(maze, shortestDirections, knownOnly,
                                    diagEnabled);
  return true; /* 成功 */
}
void SearchAlgorithm::printMap(const State state, const Pose& pose) const {
  const auto& m = (state == State::IDENTIFYING_POSITION) ? idMaze : maze;
  step_map.print(m, pose.p, pose.d);
  // step_map.printFull(m, pose.p, pose.d);
}
bool SearchAlgorithm::findShortestCandidates(Positions& candidates,
                                             const Pose& currentPose
                                             __attribute__((unused))) {
#if 0
  /* 全探索 */
  candidates.clear();
  step_map.update(maze, maze.getGoals(), false, true);
  for (int8_t x = 0; x < MAZE_SIZE; ++x)
    for (int8_t y = 0; y < MAZE_SIZE; ++y) {
      const auto p = Position(x, y);
      if (step_map.getStep(p) != StepMap::STEP_MAX && maze.unknownCount(p))
        candidates.push_back(p);
    }
  return true;
#endif
#if 0
  /* スラロームコスト考慮 */
  {
    candidates.clear();
    const bool diagEnabled = true;
    const StepMapSlalom::EdgeCost edgeCost;
    const bool knownOnly = false;
    Directions shortestDirections =
        step_map_slalom.calcShortestDirections(maze, edgeCost, knownOnly);
    if (shortestDirections.empty())
      return false; /*< failed */
    StepMap::appendStraightDirections(maze, shortestDirections, knownOnly,
                                      diagEnabled);
    auto p = maze.getStart();
    for (const auto d : shortestDirections) {
      if (!maze.isKnown(p, d))
        candidates.push_back(p);
      p = p.next(d);
    }
    return true;
  }
#endif
  /* 初期化 */
  candidates.clear();
  const auto knownOnly = false;
  /* no diag */
  {
#if 0
    /* 現状の最短経路の導出 */
    step_map.calcShortestDirections(maze, true, false);
    const auto step_current_min = step_map.getStep(maze.getStart());
#endif
    /* 最短経路の導出 */
    Directions shortestDirections =
        step_map.calcShortestDirections(maze, knownOnly, false);
    if (shortestDirections.empty())
      return false; /*< 失敗 */
    // step_map.print(maze), getc(stdin);
    /* ゴール区画内を行けるところまで直進する */
    StepMap::appendStraightDirections(maze, shortestDirections, knownOnly,
                                      false);
    /* 最短経路中の未知壁区画を訪問候補に追加 */
    auto p = maze.getStart();
    for (const auto d : shortestDirections) {
      if (!maze.isKnown(p, d))
        candidates.push_back(p);
      p = p.next(d);
    }
#if 0
    /* 現在地からゴールまでの最短経路の未知区画も追加 */
    Pose end;
    shortestDirections = step_map.getStepDownDirections(maze, currentPose, end,
                                                   knownOnly, false);
    p = currentPose.p;
    for (const auto d : shortestDirections) {
      if (!maze.isKnown(p, d))
        candidates.push_back(p);
      p = p.next(d);
    }
#endif
#if 0
    /* スタートとゴールの合計 */
    if (step_current_min != StepMap::STEP_MAX) {
      StepMap step_map_start;
      step_map_start.update(maze, {maze.getStart()}, knownOnly, false);
      for (int i = 0; i < Position::SIZE; ++i) {
        const auto p = Position::getPositionFromIndex(i);
        if (maze.unknownCount(p)) {
          const auto step = step_map.getStep(p) + step_map_start.getStep(p);
          if (step <= step_current_min)
            candidates.push_back(p);
        }
      }
    }
#endif
  }
  /* 壁ベースの斜めの最短経路上の未知区画を追加 */
  {
    /* 最短経路の導出 */
    Directions shortestDirections =
        step_map_wall.calcShortestDirections(maze, knownOnly, false);
    if (shortestDirections.empty())
      return false; /*< 失敗 */
    /* ゴール区画内の直進を追加 */
    StepMapWall::appendStraightDirections(maze, shortestDirections);
    /* 区画ベースに変換 */
    shortestDirections =
        StepMapWall::convertWallIndexDirectionsToPositionDirections(
            shortestDirections);
    /* 最短経路中の未知壁区画を訪問候補に追加 */
    auto p = maze.getStart();
    for (const auto d : shortestDirections) {
      if (!maze.isKnown(p, d))
        candidates.push_back(p);
      p = p.next(d);
    }
  }
  return true; /*< 成功 */
}
int SearchAlgorithm::countIdentityCandidates(const WallRecords& idWallRecords,
                                             Pose& ans) const {
  const int min_diff = 6; /*< 許容食い違い壁数 */
  /* 既知迷路の大きさを取得 */
  const int8_t outside_margin = 2;  //< 既知のエリアの外側に行く可能性があるので
  const int8_t max_x = std::min(MAZE_SIZE, maze.getMaxX() + 1 + outside_margin);
  const int8_t max_y = std::min(MAZE_SIZE, maze.getMaxY() + 1 + outside_margin);
  /* パターンマッチング開始 */
  int candidates_count = 0; /*< マッチ数 */
  for (int8_t x = 0; x < max_x; ++x)
    for (int8_t y = 0; y < max_y; ++y) {
      const auto offset_p = Position(x, y);
      for (const auto offset_d : Direction::Along4) {
        /* 既知壁との食い違い数を数える */
        int diffs = 0; /*< idWallRecords のうち，既知の食い違いの壁の数を格納 */
        for (const auto wr : idWallRecords) {
          const auto maze_p =
              (wr.getPosition() - idOffset).rotate(offset_d) + offset_p;
          const auto maze_d = wr.d + offset_d;
          /* 既知範囲外は除外．探索中だとちょっと危険な処理． */
          if (static_cast<uint8_t>(maze_p.x) >= max_x ||
              static_cast<uint8_t>(maze_p.y) >= max_y) {
            /* (x < 0 || x >= max_x || y < 0 || y >= max_y) の高速化 */
            diffs = 9999;
            break;
          }
          /* 既知かつ食い違い壁をカウント */
          if (maze.isKnown(maze_p, maze_d) &&
              maze.isWall(maze_p, maze_d) != wr.b)
            ++diffs;
          /* 打ち切り */
          if (diffs > min_diff)
            break;
        }
        /* 非一致条件 */
        if (diffs > min_diff)
          continue;
        /* 一致 */
        ans.p = offset_p;
        ans.d = offset_d;
        ++candidates_count;
        /* 高速化のための打ち切り */
        if (candidates_count > 1)
          return candidates_count;
      }
    }
  return candidates_count;
}
const Directions SearchAlgorithm::findMatchDirectionCandidates(
    const Position currentPosition,
    const Pose& target) const {
  const int min_diff = 0; /*< 許容食い違い壁数 */
  /* パターンマッチング開始 */
  Directions result_dirs;  //< target と一致する方向の候補を格納する
  for (const auto offset_d : Direction::Along4) {
    /* 既知壁との食い違い数を数える */
    int diffs = 0; /*< idWallRecords のうち，既知の食い違いの壁の数を格納 */
    for (const auto wr : idMaze.getWallRecords()) {
      const auto maze_p =
          target.p + (wr.getPosition() - currentPosition).rotate(offset_d);
      const auto maze_d = wr.d + offset_d;
      /* 既知かつ食い違い壁をカウント */
      if (maze.isKnown(maze_p, maze_d) && maze.isWall(maze_p, maze_d) != wr.b)
        ++diffs;
      /* 打ち切り */
      if (diffs > min_diff)
        break;
    }
    /* 非一致条件 */
    if (diffs > min_diff)
      continue;
    /* 一致 */
    result_dirs.push_back(target.d - offset_d);
  }
  return result_dirs;
}
const Position SearchAlgorithm::calcNextDirectionsInAdvance(
    Maze& maze,
    const Positions& dest,
    const Pose& startPose,
    SearchAlgorithm::NextDirections& nextDirections) {
  /* 既知区間移動方向列を生成 */
  step_map.update(maze, dest, false, false);
  const auto end = step_map.calcNextDirections(
      maze, startPose, nextDirections.nextDirectionsKnown,
      nextDirections.nextDirectionCandidates);
  /* 仮壁を立てて事前に進む候補を決定する */
  Directions nextDirectionCandidatesAdvanced;
  WallIndexes wall_backup; /*< 仮壁を立てるのでバックアップを作成 */
  while (1) {
    if (nextDirections.nextDirectionCandidates.empty())
      break;
    const Direction d =
        nextDirections.nextDirectionCandidates[0];  //< 一番行きたい方向
    nextDirectionCandidatesAdvanced.push_back(d);   //< 候補に入れる
    if (maze.isKnown(end.p, d))
      break;  //< 既知なら終わり
    /* 未知なら仮壁をたてて既知とする*/
    wall_backup.push_back(WallIndex(end.p, d));
    maze.setWall(end.p, d, true), maze.setKnown(end.p, d, true);
    /* 既知区間終了地点から次行く方向列を再計算 */
    step_map.update(maze, dest, false, false);
    nextDirections.nextDirectionCandidates =
        step_map.getNextDirectionCandidates(maze, end);
  }
  /* 仮壁を復元 */
  for (const auto i : wall_backup)
    maze.setWall(i, false), maze.setKnown(i, false);
  /* 後処理 */
  nextDirections.nextDirectionCandidates = nextDirectionCandidatesAdvanced;
  return end.p;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsSearchForGoal(
    SearchAlgorithm::NextDirections& nextDirections,
    const Pose& currentPose) {
  Positions candidates;
  for (const auto p : maze.getGoals())
    if (maze.unknownCount(p))
      candidates.push_back(p); /*< ゴール区画の未知区画を洗い出す */
  if (candidates.empty())
    return Reached;
  calcNextDirectionsInAdvance(maze, candidates, currentPose, nextDirections);
  return nextDirections.nextDirectionCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsSearchAdditionally(
    SearchAlgorithm::NextDirections& nextDirections,
    const Pose& currentPose) {
#if MAZE_DEBUG_PROFILING
  const auto t0 = microseconds();
#endif
  /* 戻り値の用意 */
  Directions& nextDirectionsKnown = nextDirections.nextDirectionsKnown;
  Directions& nextDirectionCandidates = nextDirections.nextDirectionCandidates;
  /* 最短経路上の未知区画を目的地とする */
  Positions candidates; /*< 最短経路になりうる区画 */
  if (!findShortestCandidates(candidates, currentPose))
    return Error; /*< 迷路の異常 */
  if (candidates.empty())
    return Reached; /*< 探索完了 */
  /* 既知区間移動方向列を生成 */
  step_map.update(maze, candidates, false, false);
  const auto end = step_map.calcNextDirections(
      maze, currentPose, nextDirectionsKnown, nextDirectionCandidates);
  /* 未知区間加速の判定 */
  if (!nextDirectionCandidates.empty() && nextDirectionCandidates[0] == end.d) {
    /* 直進が2連続か確認 */
    const auto nnd_candidates =
        step_map.getNextDirectionCandidates(maze, end.next(end.d));
    if (!nnd_candidates.empty() && nnd_candidates[0] == end.d)
      nextDirections.unknownAccelFlag = true;
  }
  /* 未知区間の移動優先順位を生成．仮壁を立てて事前候補を改良する */
  Directions nextDirectionCandidatesAdvanced;
  WallIndexes wall_backup; /*< 仮壁を立てるのでバックアップを作成 */
  while (1) {
    if (nextDirectionCandidates.empty())
      break;
    const Direction d = nextDirectionCandidates[0];  //< 一番行きたい方向
    nextDirectionCandidatesAdvanced.push_back(d);    //< 候補に入れる
    /* 既知なら終わり */
    if (maze.isKnown(end.p, d))
      break;
    /* 未知ならバックアップして壁を立てる*/
    wall_backup.push_back(WallIndex(end.p, d));
    maze.setWall(end.p, d, true);
    /* 袋小路なら後方を追加して終わり */
    if (maze.wallCount(end.p) == 3) {
      nextDirectionCandidatesAdvanced.push_back(
          Direction(end.d + Direction::Back));
      break;
    }
    /* 最短になりうる区画の洗い出し */
    if (!findShortestCandidates(candidates, currentPose))
      break;  //< 最短経路がなくなった場合終わり
    /* 既知区間終了地点から次行く方向列を再計算 */
    if (candidates.empty())  //< 最短候補が空の場合はスタート区画への帰還を想定
      step_map.update(maze, {maze.getStart()}, false, false);
    else
      step_map.update(maze, candidates, false, false);
    /* 次に行きたい候補を取得する */
    nextDirectionCandidates = step_map.getNextDirectionCandidates(maze, end);
  }
  /* 仮壁を復元 */
  for (const auto i : wall_backup)
    maze.setWall(i, false);
#if STEP_MAP_RECOVERY_DEBUG_MODE
  /* 表示用に仮壁を立てる前のステップを再計算 */
  findShortestCandidates(candidates, currentPose);
  step_map.update(maze, candidates, false, false);
#warning "this is debug mode!"
#endif
  /* 後処理 */
  nextDirectionCandidates = nextDirectionCandidatesAdvanced;
#if MAZE_DEBUG_PROFILING
  const auto t1 = microseconds();
  const auto dur = t1 - t0;
  static auto dur_max = dur;
  if (dur > dur_max) {
    dur_max = dur;
    maze_logi << __func__ << "\t" << dur << " us" << std::endl;
  }
#endif
  return nextDirectionCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsBackingToStart(
    SearchAlgorithm::NextDirections& nextDirections,
    const Pose& currentPose) {
  auto& nextDirectionsKnown = nextDirections.nextDirectionsKnown;
  /* 最短経路で帰れる場合はそれで帰る */
  nextDirections.nextDirectionCandidates.clear();
  nextDirections.nextDirectionsKnown = step_map.calcShortestDirections(
      maze, currentPose.p, {maze.getStart()}, true, false);
  /* できれば停止なしで帰りたい */
  const auto d_back = Direction(currentPose.d + Direction::Back);
  const auto wall_backup = maze.isWall(currentPose.p, d_back);
  maze.setWall(currentPose.p, d_back, true); /*< 後ろを一時的に塞ぐ */
  const auto tmp_nextDirectionsKnown = step_map.calcShortestDirections(
      maze, currentPose.p, {maze.getStart()}, true, false);
  maze.setWall(currentPose.p, d_back, wall_backup); /*< 壁を戻す */
  /* 経路がある，かつ，そこまで遠回りにならないとき */
  if (tmp_nextDirectionsKnown.size() &&
      tmp_nextDirectionsKnown.size() < nextDirectionsKnown.size() + 9)
    nextDirectionsKnown = tmp_nextDirectionsKnown;
  /* 最短経路で帰れる場合 */
  if (!nextDirectionsKnown.empty())
    return Reached;
  /* 行程に未知壁がある */
  const auto end_p = calcNextDirectionsInAdvance(maze, {maze.getStart()},
                                                 currentPose, nextDirections);
  if (end_p == maze.getStart())
    return Reached;
  return nextDirections.nextDirectionCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsGoingToGoal(
    SearchAlgorithm::NextDirections& nextDirections,
    const Pose& currentPose) {
  /* アクセス用の変数を用意 */
  const auto& goals = maze.getGoals();
  auto& nextDirectionCandidates = nextDirections.nextDirectionCandidates;
  auto& nextDirectionsKnown = nextDirections.nextDirectionsKnown;
  /* ゴール判定 */
  const auto it = std::find(goals.cbegin(), goals.cend(), currentPose.p);
  if (it != goals.cend())
    return Reached;
  /* 既知経路で行ける場合はそれで行く */
  nextDirectionsKnown =
      step_map.calcShortestDirections(maze, currentPose.p, goals, true, false);
  if (!nextDirectionsKnown.empty()) {
    nextDirectionCandidates = Directions{{
        Direction(nextDirectionsKnown.back() + Direction::Front),
        Direction(nextDirectionsKnown.back() + Direction::Left),
        Direction(nextDirectionsKnown.back() + Direction::Right),
        Direction(nextDirectionsKnown.back() + Direction::Back),
    }};
    return Processing;
  }
  /* 未知壁を含む場合 */
  calcNextDirectionsInAdvance(maze, goals, currentPose, nextDirections);
  return nextDirectionCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result
SearchAlgorithm::calcNextDirectionsPositionIdentification(
    SearchAlgorithm::NextDirections& nextDirections,
    Pose& currentPose,
    bool& isForceGoingToGoal) {
#if MAZE_DEBUG_PROFILING
  const auto t0 = microseconds();
#endif
  /* オフセットを調整する(処理はこのブロックで完結) */
  if (!idMaze.getWallRecords().empty()) {
    const int8_t min_x = idMaze.getMinX();
    const int8_t min_y = idMaze.getMinY();
    const int8_t max_x = idMaze.getMaxX();
    const int8_t max_y = idMaze.getMaxY();
    /* 既知のマップが迷路中央に来るように調整する */
    const auto offset_new =
        idOffset + Position((MAZE_SIZE - max_x - min_x - 1) / 2,
                            (MAZE_SIZE - max_y - min_y - 1) / 2);
    const auto offset_diff = offset_new - idOffset;
    idOffset = offset_new;
    currentPose.p = currentPose.p + offset_diff;  //< 自己位置を調整
    WallRecords tmp = idMaze.getWallRecords();
    idMaze.reset(false);
    for (const auto wr : tmp)
      idMaze.updateWall(wr.getPosition() + offset_diff, wr.d, wr.b);
  }
  /* 自己位置同定処理 */
  Pose ans;
  const int cnt = countIdentityCandidates(idMaze.getWallRecords(), ans);
  nextDirections.poseMatchCount = cnt;  //< 表示用
  if (cnt == 1) {
    /* 自己位置を修正する */
    const auto fixed_p = (currentPose.p - idOffset).rotate(ans.d) + ans.p;
    const auto fixed_d = currentPose.d + ans.d;
    currentPose = Pose(fixed_p, fixed_d);
    /* 自己位置同定中にゴール区画訪問済みなら，ゴール区画訪問をfalseにする */
    for (const auto maze_p : maze.getGoals())
      if (idMaze.unknownCount((maze_p - ans.p).rotate(-ans.d) + idOffset) == 0)
        isForceGoingToGoal = false;
    /* 自己位置同定中にスタート区画訪問済みなら，ゴール区画訪問をtrueにする */
    const auto id_start_p = (maze.getStart() - ans.p).rotate(-ans.d) + idOffset;
    if (idMaze.unknownCount(id_start_p) == 0)
      isForceGoingToGoal = true;
    /* 移動不能にならないように，真下の壁を更新する */
    maze.updateWall(currentPose.p, currentPose.d + Direction::Back, false);
    /* 自己位置同定中に未知壁を見ていたら更新する */
    const int ignore_first_walls = 12; /*< 復帰直後は壁の読み間違いがありそう */
    const auto& wallRecords = idMaze.getWallRecords();
    for (int i = ignore_first_walls; i < (int)wallRecords.size(); ++i) {
      const auto wr = wallRecords[i];
      const auto maze_p = (wr.getPosition() - idOffset).rotate(ans.d) + ans.p;
      const auto maze_d = wr.d + ans.d;
      if (!maze.isKnown(maze_p, maze_d))
        maze.updateWall(maze_p, maze_d, wr.b);
    }
    return Reached;
  } else if (cnt == 0) {
    return Error;
  }
  /* 探索方向の決定 */
  int8_t min_x = std::max(idMaze.getMinX() - 2, 0);
  int8_t min_y = std::max(idMaze.getMinY() - 2, 0);
  int8_t max_x = std::min(idMaze.getMaxX() + 2, MAZE_SIZE);
  int8_t max_y = std::min(idMaze.getMaxY() + 2, MAZE_SIZE);
  /* スタート区画への訪問を避けるため，idMazeを編集する */
  WallRecords wall_backup;
  /* 周辺の探索候補を作成 */
  Positions candidates;
  for (int8_t x = min_x; x < max_x; ++x)
    for (int8_t y = min_y; y < max_y; ++y) {
      const auto p = Position(x, y);
      /* スタート区画を避ける */
      const auto forbidden =
          findMatchDirectionCandidates(p, {Position(0, 1), Direction::South});
      for (const auto d : forbidden) {
        wall_backup.push_back(WallRecord(p, d, idMaze.isWall(p, d)));
        idMaze.setWall(p, d, true);
      }
      /* 禁止区画でない未知区画を訪問候補に追加する */
      if (forbidden.empty() && idMaze.unknownCount(p))
        candidates.push_back(p);
    }
  /* 初回の empty を除く */
  if (candidates.empty())
    for (int8_t x = min_x; x < max_x; ++x)
      for (int8_t y = min_y; y < max_y; ++y) {
        const auto p = Position(x, y);
        if (idMaze.unknownCount(p))
          candidates.push_back(p);
      }
  /* スタート区画を避けて導出 */
  calcNextDirectionsInAdvance(idMaze, candidates, currentPose, nextDirections);
  /* エラー防止のため来た方向を追加 */
  nextDirections.nextDirectionCandidates.push_back(currentPose.d +
                                                   Direction::Back);
  // step_map.print(idMaze, {currentPose.d}, currentPose.p);
  // getc(stdin);
  /* 迷路をもとに戻す */
  std::reverse(wall_backup.begin(), wall_backup.end()); /* 重複対策 */
  for (const auto wr : wall_backup)
    idMaze.setWall(wr.getPosition(), wr.d, wr.b);
  /* 既知情報からではスタート区画を避けられない場合 */
  if (step_map.getStep(currentPose.p) == StepMap::STEP_MAX)
    calcNextDirectionsInAdvance(idMaze, candidates, currentPose,
                                nextDirections);
#if MAZE_DEBUG_PROFILING
  const auto t1 = microseconds();
  const auto dur = t1 - t0;
  static auto dur_max = dur;
  if (dur > dur_max) {
    dur_max = dur;
    maze_logi << __func__ << "\t" << dur << " us" << std::endl;
  }
#endif
  return nextDirections.nextDirectionCandidates.empty() ? Error : Processing;
}

}  // namespace MazeLib
