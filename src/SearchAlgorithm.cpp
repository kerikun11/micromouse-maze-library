/**
 * @file SearchAlgorithm.cpp
 * @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2017-11-05
 * @copyright Copyright 2017 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include "MazeLib/SearchAlgorithm.h"

#include <algorithm>  //< for std::find

namespace MazeLib {

/**
 * @brief 追加探索状態で探索を始める(ゴールを急がない)
 */
#define SEARCHING_ADDITIONALLY_AT_START 1

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
  if (!findShortestCandidates(candidates)) return false;
  return candidates.empty();
}
bool SearchAlgorithm::isSolvable() {
  return !stepMap.calcShortestDirections(maze, false, false).empty();
}
void SearchAlgorithm::positionIdentifyingInit(Pose& currentPose) {
  /* オフセットを迷路の中央に設定 */
  idOffset = Position(MAZE_SIZE / 2, MAZE_SIZE / 2);
  currentPose = Pose(idOffset, Direction::East);
  idMaze.reset(false);  //< reset without setting start cell
  /* 自分の真下の壁を消す */
  idMaze.updateWall(currentPose.p, currentPose.d + Direction::Back, false);
}
bool SearchAlgorithm::updateWall(const State state, const Pose& pose,
                                 const bool left, const bool front,
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
bool SearchAlgorithm::updateWall(const State state, const Position p,
                                 const Direction d, const bool b) {
  auto& m = (state == IDENTIFYING_POSITION) ? idMaze : maze;
  return m.updateWall(p, d, b);
}
void SearchAlgorithm::resetLastWalls(const State state, const int num) {
  if (state == IDENTIFYING_POSITION) {
    return idMaze.resetLastWalls(num, false);
  }
  return maze.resetLastWalls(num, true);
}
void SearchAlgorithm::updatePose(CalcData& calcData, const Pose& newPose) {
  calcData.currentPose = newPose;
  if (calcData.result.state == SearchAlgorithm::IDENTIFYING_POSITION) return;
  if (!calcData.isForceGoingToGoal) return;
  const auto& goals = maze.getGoals();
  if (std::find(goals.cbegin(), goals.cend(), calcData.currentPose.p) !=
      goals.cend())
    calcData.isForceGoingToGoal = false;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirections(
    SearchAlgorithm::CalcData& calcData) {
  /* initialize */
  calcData.result.state = State::START;
  calcData.result.unknownAccelFlag = false;
  /* position identification */
  if (calcData.isPositionIdentification) {
    calcData.result.state = State::IDENTIFYING_POSITION;
    Result result = calcNextDirectionsPositionIdentification(calcData);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        calcData.isPositionIdentification = false;
        break;  //< go to next state
      case SearchAlgorithm::Error:
        MAZE_LOGE << "failed" << std::endl;
        calcData.result.state = State::IMPOSSIBLE;
        return result;
      default:
        MAZE_LOGE << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* search for goal */
  if (!SEARCHING_ADDITIONALLY_AT_START) {
    calcData.result.state = State::SEARCHING_FOR_GOAL;
    Result result = calcNextDirectionsSearchForGoal(calcData);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        break;  //< go to next state
      case SearchAlgorithm::Error:
        MAZE_LOGE << "failed" << std::endl;
        calcData.result.state = State::IMPOSSIBLE;
        return result;
      default:
        MAZE_LOGE << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* searching additionally */
  if (!calcData.isForceBackToStart) {
    calcData.result.state = State::SEARCHING_ADDITIONALLY;
    Result result = calcNextDirectionsSearchAdditionally(calcData);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        break;  //< go to next state
      case SearchAlgorithm::Error:
        MAZE_LOGE << "failed" << std::endl;
        calcData.result.state = State::IMPOSSIBLE;
        return result;
      default:
        MAZE_LOGE << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* force going to goal */
  if (calcData.isForceGoingToGoal) {
    calcData.result.state = State::GOING_TO_GOAL;
    Result result = calcNextDirectionsGoingToGoal(calcData);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        calcData.isForceGoingToGoal = false;
        break;  //< go to next state
      case SearchAlgorithm::Error:
        MAZE_LOGE << "failed" << std::endl;
        calcData.result.state = State::IMPOSSIBLE;
        return result;
      default:
        MAZE_LOGE << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* backing to start */
  calcData.result.state = State::BACKING_TO_START;
  Result result = calcNextDirectionsBackingToStart(calcData);
  switch (result) {
    case SearchAlgorithm::Processing:
      return result;
    case SearchAlgorithm::Reached:
      calcData.isForceBackToStart = false;
      break;  //< go to next state
    case SearchAlgorithm::Error:
      MAZE_LOGE << "failed" << std::endl;
      calcData.result.state = State::IMPOSSIBLE;
      return result;
    default:
      MAZE_LOGE << "invalid result" << std::endl;
      return SearchAlgorithm::Error;
  }
  /* reached start */
  calcData.result.state = State::REACHED_START;
  return result;
}
bool SearchAlgorithm::determineNextDirection(const CalcData& calcData,
                                             Direction& nextDirection) const {
  MAZE_DEBUG_PROFILING_START(0);
  const auto& maze = (calcData.result.state == State::IDENTIFYING_POSITION)
                         ? idMaze
                         : this->maze;
  auto nextDirectionCandidates = calcData.result.nextDirectionCandidates;
  const auto& pose = calcData.currentPose;
#if 1
  if (calcData.result.state == State::IDENTIFYING_POSITION) {
    /* スタート区画に行き得る方向を洗い出し */
    const auto forbidden = findMatchDirectionCandidates(
        pose.p, {Position(0, 1), Direction::South}, 0);
    /* スタート区画に行き得る方向を後ろに持っていく */
    std::sort(
        nextDirectionCandidates.begin(), nextDirectionCandidates.end(),
        [&](const Direction d1 __attribute__((unused)), const Direction d2) {
          return std::find(forbidden.cbegin(), forbidden.cend(), d2) !=
                 forbidden.cend();
        });
    /* 最後に後方を追加 (スタート区画を避けられない場合ふさがれてしまうので) */
    nextDirectionCandidates.push_back(pose.d + Direction::Back);
  }
#endif
  /* find a direction it can go in nextDirectionCandidates */
  const auto it = std::find_if(
      nextDirectionCandidates.cbegin(), nextDirectionCandidates.cend(),
      [&](const Direction next_d) { return maze.canGo(pose.p, next_d); });
  MAZE_DEBUG_PROFILING_END(0);
  if (it == nextDirectionCandidates.cend()) return false;  //< no answer
  nextDirection = *it;
  return true;
}
bool SearchAlgorithm::calcShortestDirections(
    Directions& shortestDirections, const bool diagEnabled,
    const StepMapSlalom::EdgeCost& edgeCost) {
  const bool knownOnly = true;
  if (diagEnabled) {
    shortestDirections =
        stepMapSlalom.calcShortestDirections(maze, edgeCost, knownOnly);
    if (shortestDirections.empty()) return false;  //< failed
    shortestCost = stepMapSlalom.getShortestCost();
  } else {
    shortestDirections = stepMap.calcShortestDirections(maze, knownOnly, false);
    if (shortestDirections.empty()) return false; /* no path to goal */
    shortestCost =
        stepMap.getStep(maze.getStart()) * stepMap.getScalingFactor();
  }
  StepMap::appendStraightDirections(maze, shortestDirections, knownOnly,
                                    diagEnabled);
  return true; /* 成功 */
}
void SearchAlgorithm::printStepMap(const State state, const Pose& pose) const {
  const auto& m = (state == State::IDENTIFYING_POSITION) ? idMaze : maze;
  stepMap.print(m, pose.p, pose.d);
  // stepMap.printFull(m, pose.p, pose.d);
}
bool SearchAlgorithm::findShortestCandidates(Positions& candidates,
                                             const Pose& currentPose
                                             __attribute__((unused))) {
#if 0
  /* 全探索 */
  candidates.clear();
  stepMap.update(maze, maze.getGoals(), false, true);
  for (int8_t x = 0; x < MAZE_SIZE; ++x)
    for (int8_t y = 0; y < MAZE_SIZE; ++y) {
      const auto p = Position(x, y);
      if (stepMap.getStep(p) != StepMap::STEP_MAX && maze.unknownCount(p))
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
        stepMapSlalom.calcShortestDirections(maze, edgeCost, knownOnly);
    if (shortestDirections.empty())
      return false;  //< failed
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
#if 1
  /* 斜めなし方向 */
  {
#if 0
    /* 現状の最短経路の導出 */
    stepMap.calcShortestDirections(maze, true, false);
    const auto step_current_min = stepMap.getStep(maze.getStart());
#endif
    /* 最短経路の導出 */
    Directions shortestDirections =
        stepMap.calcShortestDirections(maze, knownOnly, false);
    if (shortestDirections.empty()) return false;  //< 経路なし
    /* ゴール区画内を行けるところまで直進する */
    StepMap::appendStraightDirections(maze, shortestDirections, knownOnly,
                                      false);
    /* 最短経路中の未知壁区画を訪問候補に追加 */
    auto p = maze.getStart();
    for (const auto d : shortestDirections) {
      if (!maze.isKnown(p, d)) candidates.push_back(p);
      p = p.next(d);
    }
#if 0
    /* 現在地からゴールまでの最短経路の未知区画も追加 */
    Pose end;
    shortestDirections = stepMap.getStepDownDirections(
        maze, currentPose, end, knownOnly, false, false);
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
      StepMap stepMapFromStart;
      stepMapFromStart.update(maze, {maze.getStart()}, knownOnly, false);
      for (int i = 0; i < Position::SIZE; ++i) {
        const auto p = Position::getPositionFromIndex(i);
        if (maze.unknownCount(p)) {
          const auto step = stepMap.getStep(p) + stepMapFromStart.getStep(p);
          if (step <= step_current_min)
            candidates.push_back(p);
        }
      }
    }
#endif
  }
#endif
#if 1
  /* 壁ベースの斜めの最短経路上の未知区画を追加 */
  {
    /* 最短経路の導出 */
    Directions shortestDirections =
        stepMapWall.calcShortestDirections(maze, knownOnly, false);
    if (shortestDirections.empty()) return false;  //< 経路なし
    /* ゴール区画内の直進を追加 */
    StepMapWall::appendStraightDirections(maze, shortestDirections);
    /* 最短経路中の未知壁区画を訪問候補に追加 */
    auto i = StepMapWall::START_WALL_INDEX;
    for (const auto d : shortestDirections) {
      if (!maze.isKnown(i)) candidates.push_back(i.getPosition());
      i = i.next(d);
    }
  }
#endif
  return true;  //< 成功
}
int SearchAlgorithm::countIdentityCandidates(const WallRecords& idWallRecords,
                                             Pose& ans) const {
  MAZE_DEBUG_PROFILING_START(0)
  const int min_diff = 6;  //< 許容食い違い壁数
  /* 既知迷路の大きさを取得 */
  const int8_t outside_margin = 2;  //< 既知のエリアの外側に行く可能性があるので
  const int8_t max_x = std::min(MAZE_SIZE, maze.getMaxX() + 1 + outside_margin);
  const int8_t max_y = std::min(MAZE_SIZE, maze.getMaxY() + 1 + outside_margin);
  /* パターンマッチング開始 */
  int candidates_count = 0;  //< マッチ数
  for (int8_t x = 0; x < max_x; ++x)
    for (int8_t y = 0; y < max_y; ++y) {
      const auto offset_p = Position(x, y);
      for (const auto offset_d : Direction::Along4()) {
        /* 既知壁との食い違い数を数える */
        int diffs = 0;  //< idWallRecords のうち、既知の食い違いの壁の数を格納
        for (const auto wr : idWallRecords) {
          const auto maze_p =
              (wr.getPosition() - idOffset).rotate(offset_d) + offset_p;
          const auto maze_d = wr.d + offset_d;
          /* 既知範囲外は除外。探索中だとちょっと危険な処理。 */
          if (static_cast<uint8_t>(maze_p.x) >= max_x ||
              static_cast<uint8_t>(maze_p.y) >= max_y) {
            /* (x < 0 || x >= max_x || y < 0 || y >= max_y) の高速化 */
            diffs = min_diff + 1;
            break;
          }
          /* 既知かつ食い違い壁をカウント */
          const auto maze_wi = WallIndex(maze_p, maze_d);
          if (maze.isKnown(maze_wi) && maze.isWall(maze_wi) != wr.b) ++diffs;
          /* 打ち切り */
          if (diffs > min_diff) break;
        }
        /* 非一致条件 */
        if (diffs > min_diff) continue;
        /* 一致 */
        ans.p = offset_p;
        ans.d = offset_d;
        ++candidates_count;
        /* 高速化のための打ち切り */
        if (candidates_count > 1) {
          MAZE_DEBUG_PROFILING_END(0)
          return candidates_count;
        }
      }
    }
  MAZE_DEBUG_PROFILING_END(0)
  return candidates_count;
}
const Directions SearchAlgorithm::findMatchDirectionCandidates(
    const Position currentPosition, const Pose& target,
    const int ignoreWalls) const {
  MAZE_DEBUG_PROFILING_START(0)
  const int min_diff = ignoreWalls;  //< 許容食い違い壁数
  /* パターンマッチング開始 */
  Directions result_dirs;  //< target と一致する方向の候補を格納する
  for (const auto offset_d : Direction::Along4()) {
    /* 既知壁との食い違い数を数える */
    int diffs = 0;  //< idWallRecords のうち、既知の食い違いの壁の数を格納
    for (const auto wr : idMaze.getWallRecords()) {
      const auto maze_p =
          target.p + (wr.getPosition() - currentPosition).rotate(offset_d);
      const auto maze_d = wr.d + offset_d;
      /* 既知かつ食い違い壁をカウント */
      if (maze.isKnown(maze_p, maze_d) && maze.isWall(maze_p, maze_d) != wr.b)
        ++diffs;
      /* 打ち切り */
      if (diffs > min_diff) break;
    }
    /* 非一致条件 */
    if (diffs > min_diff) continue;
    /* 一致 */
    result_dirs.push_back(target.d - offset_d);
  }
  MAZE_DEBUG_PROFILING_END(0)
  return result_dirs;
}
const Pose SearchAlgorithm::calcNextDirectionsInAdvance(
    Maze& maze, const Positions& dest, SearchAlgorithm::CalcData& calcData) {
  /* 既知区間移動方向列を生成 */
  stepMap.update(maze, dest, false, false);
  const auto end = stepMap.calcNextDirections(
      maze, calcData.currentPose, calcData.result.nextDirectionsKnown,
      calcData.result.nextDirectionCandidates);
  /* 仮壁を立てて事前に進む候補を決定する */
  Directions nextDirectionCandidatesAdvanced;
  WallIndexes wall_backup;  //< 仮壁を立てるのでバックアップを作成
  while (1) {
    if (calcData.result.nextDirectionCandidates.empty()) break;
    const Direction d =
        calcData.result.nextDirectionCandidates[0];  //< 一番行きたい方向
    nextDirectionCandidatesAdvanced.push_back(d);    //< 候補に入れる
    if (maze.isKnown(end.p, d)) break;               //< 既知なら終わり
    /* 未知なら仮壁をたてて既知とする*/
    wall_backup.push_back(WallIndex(end.p, d));
    maze.setWall(end.p, d, true), maze.setKnown(end.p, d, true);
    /* 既知区間終了地点から次行く方向列を再計算 */
    stepMap.update(maze, dest, false, false);
    calcData.result.nextDirectionCandidates =
        stepMap.getNextDirectionCandidates(maze, end);
  }
  /* 仮壁を復元 */
  for (const auto i : wall_backup)
    maze.setWall(i, false), maze.setKnown(i, false);
  /* 後処理 */
  calcData.result.nextDirectionCandidates = nextDirectionCandidatesAdvanced;
  return end;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsSearchForGoal(
    SearchAlgorithm::CalcData& calcData) {
  Positions candidates;
  for (const auto p : maze.getGoals())
    if (maze.unknownCount(p))
      candidates.push_back(p);  //< ゴール区画の未知区画を洗い出す
  if (candidates.empty()) return Reached;
  calcNextDirectionsInAdvance(maze, candidates, calcData);
  return calcData.result.nextDirectionCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsSearchAdditionally(
    SearchAlgorithm::CalcData& calcData) {
  MAZE_DEBUG_PROFILING_START(0)
  Pose& currentPose = calcData.currentPose;
  /* 戻り値の用意 */
  Directions& nextDirectionsKnown = calcData.result.nextDirectionsKnown;
  Directions& nextDirectionCandidates = calcData.result.nextDirectionCandidates;
  /* 最短経路上の未知区画を目的地とする */
  Positions candidates;  //< 最短経路になりうる区画
  if (!findShortestCandidates(candidates, currentPose)) {
    MAZE_LOGE << "failed" << std::endl;
    return Error;  //< 迷路の異常
  }
  if (candidates.empty()) return Reached;  //< 探索完了
  /* 既知区間移動方向列を生成 */
  stepMap.update(maze, candidates, false, false);
  const auto end = stepMap.calcNextDirections(
      maze, currentPose, nextDirectionsKnown, nextDirectionCandidates);
  /* 未知区間加速の判定 */
  if (!nextDirectionCandidates.empty() && nextDirectionCandidates[0] == end.d) {
    /* 直進が2連続か確認 */
    const auto nnd_candidates =
        stepMap.getNextDirectionCandidates(maze, end.next(end.d));
    if (!nnd_candidates.empty() && nnd_candidates[0] == end.d)
      calcData.result.unknownAccelFlag = true;
  }
  /* 未知区間の移動優先順位を生成。仮壁を立てて事前候補を改良する */
  Directions nextDirectionCandidatesAdvanced;
  WallIndexes wall_backup;  //< 仮壁を立てるのでバックアップを作成
  while (1) {
    if (nextDirectionCandidates.empty()) break;
    const Direction d = nextDirectionCandidates[0];  //< 一番行きたい方向
    nextDirectionCandidatesAdvanced.push_back(d);    //< 候補に入れる
    /* 既知なら終わり */
    if (maze.isKnown(end.p, d)) break;
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
      stepMap.update(maze, {maze.getStart()}, false, false);
    else
      stepMap.update(maze, candidates, false, false);
    /* 次に行きたい候補を取得する */
    nextDirectionCandidates = stepMap.getNextDirectionCandidates(maze, end);
  }
  /* 仮壁を復元 */
  for (const auto i : wall_backup) maze.setWall(i, false);
#if 0
  /* 表示用に仮壁を立てる前のステップを再計算 */
  findShortestCandidates(candidates, currentPose);
  stepMap.update(maze, candidates, false, false);
#warning "this is debug mode!"
#endif
  /* 後処理 */
  nextDirectionCandidates = nextDirectionCandidatesAdvanced;
  MAZE_DEBUG_PROFILING_END(0)
  if (nextDirectionCandidates.empty()) {
    MAZE_LOGE << "failed" << std::endl;
  }
  return nextDirectionCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsBackingToStart(
    SearchAlgorithm::CalcData& calcData) {
  auto& nextDirectionsKnown = calcData.result.nextDirectionsKnown;
  auto& currentPose = calcData.currentPose;
  /* 最短経路で帰れる場合はそれで帰る */
  calcData.result.nextDirectionCandidates.clear();
  calcData.result.nextDirectionsKnown = stepMap.calcShortestDirections(
      maze, currentPose.p, {maze.getStart()}, true, false);
  /* できれば停止なしで帰りたい */
  const auto d_back = Direction(currentPose.d + Direction::Back);
  const auto wall_backup = maze.isWall(currentPose.p, d_back);
  maze.setWall(currentPose.p, d_back, true);  //< 後ろを一時的に塞ぐ
  const auto tmp_nextDirectionsKnown = stepMap.calcShortestDirections(
      maze, currentPose.p, {maze.getStart()}, true, false);
  maze.setWall(currentPose.p, d_back, wall_backup);  //< 壁を戻す
  /* 経路がある、かつ、そこまで遠回りにならないとき */
  if (tmp_nextDirectionsKnown.size() &&
      tmp_nextDirectionsKnown.size() < nextDirectionsKnown.size() + 9)
    nextDirectionsKnown = tmp_nextDirectionsKnown;
  /* 最短経路で帰れる場合 */
  if (!nextDirectionsKnown.empty()) return Reached;
  /* 行程に未知壁がある */
  const auto end_p =
      calcNextDirectionsInAdvance(maze, {maze.getStart()}, calcData).p;
  if (end_p == maze.getStart()) return Reached;
  return calcData.result.nextDirectionCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsGoingToGoal(
    SearchAlgorithm::CalcData& calcData) {
  auto& currentPose = calcData.currentPose;
  /* アクセス用の変数を用意 */
  const auto& goals = maze.getGoals();
  auto& nextDirectionCandidates = calcData.result.nextDirectionCandidates;
  auto& nextDirectionsKnown = calcData.result.nextDirectionsKnown;
  /* ゴール判定 */
  const auto it = std::find(goals.cbegin(), goals.cend(), currentPose.p);
  if (it != goals.cend()) return Reached;
  /* 既知経路で行ける場合はそれで行く */
  nextDirectionsKnown =
      stepMap.calcShortestDirections(maze, currentPose.p, goals, true, false);
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
  calcNextDirectionsInAdvance(maze, goals, calcData);
  return nextDirectionCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result
SearchAlgorithm::calcNextDirectionsPositionIdentification(
    SearchAlgorithm::CalcData& calcData) {
  MAZE_DEBUG_PROFILING_START(0)
  auto& currentPose = calcData.currentPose;
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
  calcData.result.positionIdMatchedCount = cnt;  //< 表示用
  if (cnt == 1) {
    /* 自己位置を修正する */
    const auto fixed_p = (currentPose.p - idOffset).rotate(ans.d) + ans.p;
    const auto fixed_d = currentPose.d + ans.d;
    currentPose = Pose(fixed_p, fixed_d);
    /* 自己位置同定中にゴール区画訪問済みなら、ゴール区画訪問をfalseにする */
    for (const auto maze_p : maze.getGoals())
      if (idMaze.unknownCount((maze_p - ans.p).rotate(-ans.d) + idOffset) == 0)
        calcData.isForceGoingToGoal = false;
    /* 自己位置同定中にスタート区画訪問済みなら、ゴール区画訪問をtrueにする */
    const auto id_start_p = (maze.getStart() - ans.p).rotate(-ans.d) + idOffset;
    if (idMaze.unknownCount(id_start_p) == 0)
      calcData.isForceGoingToGoal = true;
    /* 移動不能にならないように、真下の壁を更新する */
    maze.updateWall(currentPose.p, currentPose.d + Direction::Back, false);
    /* 自己位置同定中に未知壁を見ていたら更新する */
    const int ignore_first_walls = 12;  //< 復帰直後は壁の読み間違いがありそう
    const auto& wallRecords = idMaze.getWallRecords();
    for (int i = ignore_first_walls; i < static_cast<int>(wallRecords.size());
         ++i) {
      const auto wr = wallRecords[i];
      const auto maze_p = (wr.getPosition() - idOffset).rotate(ans.d) + ans.p;
      const auto maze_d = wr.d + ans.d;
      if (!maze.isKnown(maze_p, maze_d)) maze.updateWall(maze_p, maze_d, wr.b);
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
  /* スタート区画への訪問を避けるため、idMazeを編集する */
  WallRecords wall_backup;
  /* 周辺の探索候補を作成 */
  Positions candidates;
  for (int8_t x = min_x; x < max_x; ++x)
    for (int8_t y = min_y; y < max_y; ++y) {
      const auto p = Position(x, y);
      /* スタート区画を避ける */
      const auto forbidden = findMatchDirectionCandidates(
          p, {Position(0, 1), Direction::South}, 0);
      for (const auto d : forbidden) {
        wall_backup.push_back(WallRecord(p, d, idMaze.isWall(p, d)));
        idMaze.setWall(p, d, true);
      }
      /* 禁止区画でない未知区画を訪問候補に追加する */
      if (forbidden.empty() && idMaze.unknownCount(p)) candidates.push_back(p);
    }
  /* 初回の empty を除く */
  if (candidates.empty())
    for (int8_t x = min_x; x < max_x; ++x)
      for (int8_t y = min_y; y < max_y; ++y)
        if (const auto p = Position(x, y); idMaze.unknownCount(p))
          candidates.push_back(p);
  /* スタート区画を避けたことで閉じ込められたか確認 */
  stepMap.update(idMaze, candidates, false, false);
  const auto current_step =
      stepMap.getStep(currentPose.p.next(currentPose.d + Direction::Back));
  if (current_step != StepMap::STEP_MAX) {
    /* スタート区画を避けて導出 */
    calcNextDirectionsInAdvance(idMaze, candidates, calcData);
    // stepMap.print(idMaze, {currentPose.d}, currentPose.p);
    // MAZE_LOGI << std::endl;
    // getc(stdin);
  }
  /* 迷路をもとに戻す */
  std::reverse(wall_backup.begin(), wall_backup.end()); /* 重複対策 */
  for (const auto wr : wall_backup)
    idMaze.setWall(wr.getPosition(), wr.d, wr.b);
  /* 既知情報からではスタート区画を避けられない場合は壁を戻してから計算 */
  if (current_step == StepMap::STEP_MAX) {
    // calcNextDirectionsInAdvance(idMaze, candidates, calcData);
    calcData.result.nextDirectionsKnown.clear();
    calcData.result.nextDirectionCandidates = {
        Direction(currentPose.d + Direction::Front),
        Direction(currentPose.d + Direction::Left),
        Direction(currentPose.d + Direction::Right),
    };
    // stepMap.print(idMaze, {currentPose.d}, currentPose.p);
    // MAZE_LOGI << std::endl;
    // getc(stdin);
  }
  MAZE_DEBUG_PROFILING_END(0)
  return calcData.result.nextDirectionCandidates.empty() ? Error : Processing;
}

}  // namespace MazeLib
