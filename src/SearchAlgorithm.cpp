/**
 * @file SearchAlgorithm.cpp
 * @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2017-11-05
 * @copyright Copyright 2017 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include "MazeLib/SearchAlgorithm.h"

#include <algorithm>

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
bool SearchAlgorithm::isComplete() {
  Positions candidates;
  if (!findShortestCandidates(candidates))
    return false;
  return candidates.empty();
}
bool SearchAlgorithm::isSolvable() {
  return !step_map.calcShortestDirections(maze, false, false).empty();
}
void SearchAlgorithm::positionIdentifyingInit(Pose& current_pose) {
  /* オフセットを迷路の中央に設定 */
  idOffset = Position(MAZE_SIZE / 2, MAZE_SIZE / 2);
  current_pose = Pose(idOffset, Direction::East);
  idMaze.reset(false); /*< reset without setting start cell */
  /* 自分の真下の壁を消す */
  idMaze.updateWall(current_pose.p, current_pose.d + Direction::Back, false);
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
                                 const Position& p,
                                 const Direction d,
                                 const bool b) {
  auto& m = (state == IDENTIFYING_POSITION) ? idMaze : maze;
  return m.updateWall(p, d, b);
}
void SearchAlgorithm::resetLastWalls(const State state, const int num) {
  auto& m = (state == IDENTIFYING_POSITION) ? idMaze : maze;
  return m.resetLastWalls(num);
}
void SearchAlgorithm::updatePose(const State& state,
                                 Pose& current_pose,
                                 bool& isForceGoingToGoal) {
  if (state == SearchAlgorithm::IDENTIFYING_POSITION)
    return;
  if (!isForceGoingToGoal)
    return;
  const auto& goals = maze.getGoals();
  if (std::find(goals.cbegin(), goals.cend(), current_pose.p) != goals.cend())
    isForceGoingToGoal = false;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirections(
    SearchAlgorithm::NextDirections& next_directions,
    Pose& current_pose,
    bool& isPositionIdentifying,
    bool& isForceBackToStart,
    bool& isForceGoingToGoal) {
  /* initialize */
  next_directions.state = State::START;
  next_directions.unknown_accel_flag = false;
  /* position identification */
  if (isPositionIdentifying) {
    next_directions.state = State::IDENTIFYING_POSITION;
    Result result = calcNextDirectionsPositionIdentification(
        next_directions, current_pose, isForceGoingToGoal);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        isPositionIdentifying = false;
        break; /*< go to next state */
      case SearchAlgorithm::Error:
        next_directions.state = State::IMPOSSIBLE;
        return result;
      default:
        maze_loge << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* search for goal */
  if (!SEARCHING_ADDITIONALLY_AT_START) {
    next_directions.state = State::SEARCHING_FOR_GOAL;
    Result result =
        calcNextDirectionsSearchForGoal(next_directions, current_pose);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        break; /*< go to next state */
      case SearchAlgorithm::Error:
        next_directions.state = State::IMPOSSIBLE;
        return result;
      default:
        maze_loge << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* searching additionally */
  if (!isForceBackToStart) {
    next_directions.state = State::SEARCHING_ADDITIONALLY;
    Result result =
        calcNextDirectionsSearchAdditionally(next_directions, current_pose);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        break; /*< go to next state */
      case SearchAlgorithm::Error:
        next_directions.state = State::IMPOSSIBLE;
        return result;
      default:
        maze_loge << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* force going to goal */
  if (isForceGoingToGoal) {
    next_directions.state = State::GOING_TO_GOAL;
    Result result =
        calcNextDirectionsGoingToGoal(next_directions, current_pose);
    switch (result) {
      case SearchAlgorithm::Processing:
        return result;
      case SearchAlgorithm::Reached:
        return SearchAlgorithm::Processing;  //< to reach the goal exactly
      case SearchAlgorithm::Error:
        next_directions.state = State::IMPOSSIBLE;
        return result;
      default:
        maze_loge << "invalid result" << std::endl;
        return SearchAlgorithm::Error;
    }
  }
  /* backing to start */
  next_directions.state = State::BACKING_TO_START;
  Result result =
      calcNextDirectionsBackingToStart(next_directions, current_pose);
  switch (result) {
    case SearchAlgorithm::Processing:
      return result;
    case SearchAlgorithm::Reached:
      isForceBackToStart = false;
      break; /*< go to next state */
    case SearchAlgorithm::Error:
      next_directions.state = State::IMPOSSIBLE;
      return result;
    default:
      maze_loge << "invalid result" << std::endl;
      return SearchAlgorithm::Error;
  }
  /* reached start */
  next_directions.state = State::REACHED_START;
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
    Directions& shortest_dirs,
    const bool diag_enabled,
    const StepMapSlalom::EdgeCost& edge_cost) {
  const bool known_only = true;
  if (!step_map_slalom.calcShortestDirections(maze, edge_cost, shortest_dirs,
                                              known_only, diag_enabled))
    return false; /* failed */
  StepMap::appendStraightDirections(maze, shortest_dirs, known_only,
                                    diag_enabled);
  return true; /* 成功 */
}
void SearchAlgorithm::printMap(const State state, const Pose& pose) const {
  const auto& m = (state == State::IDENTIFYING_POSITION) ? idMaze : maze;
  step_map.print(m, pose.p, pose.d);
  // step_map.printFull(m, pose.p, pose.d);
}
bool SearchAlgorithm::findShortestCandidates(Positions& candidates,
                                             const Pose& current_pose) {
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
  candidates.clear();
  for (const auto diag_enabled : {true, false}) {
    Directions shortest_dirs;
    const StepMapSlalom::EdgeCost edge_cost;
    const bool known_only = false;
    if (!step_map_slalom.calcShortestDirections(maze, edge_cost, shortest_dirs,
                                                known_only, diag_enabled))
      return false; /* failed */
    StepMap::appendStraightDirections(maze, shortest_dirs, known_only,
                                      diag_enabled);
    auto p = maze.getStart();
    for (const auto d : shortest_dirs) {
      if (!maze.isKnown(p, d))
        candidates.push_back(p);
      p = p.next(d);
    }
  }
  return true;
#endif
  candidates.clear();
  const auto known_only = false;
  /* no diag */
  {
    /* 最短経路の導出 */
    Directions shortest_dirs =
        step_map.calcShortestDirections(maze, known_only, false);
    if (shortest_dirs.empty())
      return false; /*< 失敗 */
    /* ゴール区画内を行けるところまで直進する */
    StepMap::appendStraightDirections(maze, shortest_dirs, known_only, false);
    /* 最短経路中の未知壁区画を訪問候補に追加 */
    auto p = maze.getStart();
    for (const auto d : shortest_dirs) {
      if (!maze.isKnown(p, d))
        candidates.push_back(p);
      p = p.next(d);
    }
#if 1
    /* 現在地からゴールまでの最短経路の未知区画も追加 */
    Pose end;
    shortest_dirs = step_map.getStepDownDirections(maze, current_pose, end,
                                                   known_only, false);
    p = current_pose.p;
    for (const auto d : shortest_dirs) {
      if (!maze.isKnown(p, d))
        candidates.push_back(p);
      p = p.next(d);
    }
#endif
  }
  /* diag */
  {
    /* 最短経路の導出 */
    Directions shortest_dirs =
        step_map_wall.calcShortestDirections(maze, known_only, false);
    if (shortest_dirs.empty())
      return false; /*< 失敗 */
    /* 最短経路中の未知壁区画を訪問候補に追加 */
    auto i = WallIndex(0, 0, 1);  //< スタート区画
    for (const auto d : shortest_dirs)
      if (!maze.isKnown(i = i.next(d)))
        candidates.push_back(i.getPosition());
    /* ゴール区画内を行けるところまで直進する */
    if (shortest_dirs.size()) {
      const auto d = shortest_dirs.back();
      while (1) {
        i = i.next(d);
        if (maze.isWall(i))
          break;
        if (!maze.isKnown(i))
          candidates.push_back(i.getPosition());
      }
    }
#if 1
    /* 現在地からゴールまでの最短経路の未知区画も追加 */
    /* 斜めのこれは未知区間加速との相性悪し！！ */
    const auto start =
        WallIndex(current_pose.p, current_pose.d + Direction::Back);
    WallIndex end;
    shortest_dirs =
        step_map_wall.getStepDownDirections(maze, start, end, false, false);
    i = start;
    for (const auto d : shortest_dirs) {
      if (!maze.isKnown(i))
        candidates.push_back(i.getPosition());
      i = i.next(d);
    }
#endif
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
    const Position& cur_p,
    const Pose& target) const {
  const int min_diff = 0; /*< 許容食い違い壁数 */
  /* パターンマッチング開始 */
  Directions result_dirs;  //< target と一致する方向の候補を格納する
  for (const auto offset_d : Direction::Along4) {
    /* 既知壁との食い違い数を数える */
    int diffs = 0; /*< idWallRecords のうち，既知の食い違いの壁の数を格納 */
    for (const auto wr : idMaze.getWallRecords()) {
      const auto maze_p =
          target.p + (wr.getPosition() - cur_p).rotate(offset_d);
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
    const Pose& start_pose,
    SearchAlgorithm::NextDirections& next_directions) {
  /* 既知区間移動方向列を生成 */
  step_map.update(maze, dest, false, false);
  const auto end = step_map.calcNextDirections(
      maze, start_pose, next_directions.next_directions_known,
      next_directions.next_direction_candidates);
  /* 仮壁を立てて事前に進む候補を決定する */
  Directions nextDirectionCandidatesAdvanced;
  WallIndexes wall_backup; /*< 仮壁を立てるのでバックアップを作成 */
  while (1) {
    if (next_directions.next_direction_candidates.empty())
      break;
    const Direction d =
        next_directions.next_direction_candidates[0];  //< 一番行きたい方向
    nextDirectionCandidatesAdvanced.push_back(d);      //< 候補に入れる
    if (maze.isKnown(end.p, d))
      break;  //< 既知なら終わり
    /* 未知なら仮壁をたてて既知とする*/
    wall_backup.push_back(WallIndex(end.p, d));
    maze.setWall(end.p, d, true), maze.setKnown(end.p, d, true);
    /* 既知区間終了地点から次行く方向列を再計算 */
    step_map.update(maze, dest, false, false);
    next_directions.next_direction_candidates =
        step_map.getNextDirectionCandidates(maze, end);
  }
  /* 仮壁を復元 */
  for (const auto i : wall_backup)
    maze.setWall(i, false), maze.setKnown(i, false);
  /* 後処理 */
  next_directions.next_direction_candidates = nextDirectionCandidatesAdvanced;
  return end.p;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsSearchForGoal(
    SearchAlgorithm::NextDirections& next_directions,
    const Pose& current_pose) {
  Positions candidates;
  for (const auto p : maze.getGoals())
    if (maze.unknownCount(p))
      candidates.push_back(p); /*< ゴール区画の未知区画を洗い出す */
  if (candidates.empty())
    return Reached;
  calcNextDirectionsInAdvance(maze, candidates, current_pose, next_directions);
  return next_directions.next_direction_candidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsSearchAdditionally(
    SearchAlgorithm::NextDirections& next_directions,
    const Pose& current_pose) {
  /* 戻り値の用意 */
  Directions& nextDirectionsKnown = next_directions.next_directions_known;
  Directions& nextDirectionCandidates =
      next_directions.next_direction_candidates;
  /* 最短経路上の未知区画を目的地とする */
  Positions candidates; /*< 最短経路になりうる区画 */
  if (!findShortestCandidates(candidates, current_pose))
    return Error; /*< 迷路の異常 */
  if (candidates.empty())
    return Reached; /*< 探索完了 */
  /* 既知区間移動方向列を生成 */
  step_map.update(maze, candidates, false, false);
  const auto end = step_map.calcNextDirections(
      maze, current_pose, nextDirectionsKnown, nextDirectionCandidates);
  /* 未知区間加速の判定 */
  if (!nextDirectionCandidates.empty() && nextDirectionCandidates[0] == end.d) {
    /* 直進が2連続か確認 */
    const auto nnd_candidates =
        step_map.getNextDirectionCandidates(maze, end.next(end.d));
    if (!nnd_candidates.empty() && nnd_candidates[0] == end.d)
      next_directions.unknown_accel_flag = true;
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
    if (!findShortestCandidates(candidates, current_pose))
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
#if 0
  /* 表示用に仮壁を立てる前のステップを再計算 */
  findShortestCandidates(candidates, current_pose);
  step_map.update(maze, candidates, false, false);
#endif
  /* 後処理 */
  nextDirectionCandidates = nextDirectionCandidatesAdvanced;
  return nextDirectionCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsBackingToStart(
    SearchAlgorithm::NextDirections& next_directions,
    const Pose& current_pose) {
  auto& nextDirectionsKnown = next_directions.next_directions_known;
  /* 最短経路で帰れる場合はそれで帰る */
  next_directions.next_direction_candidates.clear();
  next_directions.next_directions_known = step_map.calcShortestDirections(
      maze, current_pose.p, {maze.getStart()}, true, false);
  /* できれば停止なしで帰りたい */
  const auto d_back = Direction(current_pose.d + Direction::Back);
  const auto wall_backup = maze.isWall(current_pose.p, d_back);
  maze.setWall(current_pose.p, d_back, true); /*< 後ろを一時的に塞ぐ */
  const auto tmp_nextDirectionsKnown = step_map.calcShortestDirections(
      maze, current_pose.p, {maze.getStart()}, true, false);
  maze.setWall(current_pose.p, d_back, wall_backup); /*< 壁を戻す */
  /* 経路がある，かつ，そこまで遠回りにならないとき */
  if (tmp_nextDirectionsKnown.size() &&
      tmp_nextDirectionsKnown.size() < nextDirectionsKnown.size() + 9)
    nextDirectionsKnown = tmp_nextDirectionsKnown;
  /* 最短経路で帰れる場合 */
  if (!nextDirectionsKnown.empty())
    return Reached;
  /* 行程に未知壁がある */
  const auto end_p = calcNextDirectionsInAdvance(maze, {maze.getStart()},
                                                 current_pose, next_directions);
  if (end_p == maze.getStart())
    return Reached;
  return next_directions.next_direction_candidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirectionsGoingToGoal(
    SearchAlgorithm::NextDirections& next_directions,
    const Pose& current_pose) {
  const auto& goals = maze.getGoals();
  /* 最短経路で帰れる場合はそれで帰る */
  next_directions.next_direction_candidates.clear();
  next_directions.next_directions_known =
      step_map.calcShortestDirections(maze, current_pose.p, goals, true, false);
  if (std::find(goals.cbegin(), goals.cend(), next_directions.known_end.p) !=
      goals.cend())
    return Reached;
  /* 未知壁を含む場合 */
  auto& nextDirectionCandidates = next_directions.next_direction_candidates;
  calcNextDirectionsInAdvance(maze, goals, current_pose, next_directions);
  /* ゴール判定 */
  const auto next_p =
      current_pose.p.next(nextDirectionCandidates[0] + Direction::Back);
  const auto it = std::find(goals.cbegin(), goals.cend(), next_p);
  if (it != goals.cend())
    return Reached;
  return nextDirectionCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result
SearchAlgorithm::calcNextDirectionsPositionIdentification(
    SearchAlgorithm::NextDirections& next_directions,
    Pose& current_pose,
    bool& isForceGoingToGoal) {
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
    current_pose.p = current_pose.p + offset_diff;  //< 自己位置を調整
    WallRecords tmp = idMaze.getWallRecords();
    idMaze.reset(false);
    for (const auto wr : tmp)
      idMaze.updateWall(wr.getPosition() + offset_diff, wr.d, wr.b);
  }
  /* 自己位置同定処理 */
  Pose ans;
  const int cnt = countIdentityCandidates(idMaze.getWallRecords(), ans);
  next_directions.match_count = cnt;  //< 表示用
  if (cnt == 1) {
    /* 自己位置を修正する */
    const auto fixed_p = (current_pose.p - idOffset).rotate(ans.d) + ans.p;
    const auto fixed_d = current_pose.d + ans.d;
    current_pose = Pose(fixed_p, fixed_d);
    /* 自己位置同定中にゴール区画訪問済みなら，ゴール区画訪問をfalseにする */
    for (const auto maze_p : maze.getGoals())
      if (idMaze.unknownCount((maze_p - ans.p).rotate(-ans.d) + idOffset) == 0)
        isForceGoingToGoal = false;
    /* 自己位置同定中にスタート区画訪問済みなら，ゴール区画訪問をtrueにする */
    const auto id_start_p = (maze.getStart() - ans.p).rotate(-ans.d) + idOffset;
    if (idMaze.unknownCount(id_start_p) == 0)
      isForceGoingToGoal = true;
    /* 移動不能にならないように，真下の壁を更新する */
    maze.updateWall(current_pose.p, current_pose.d + Direction::Back, false);
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
  calcNextDirectionsInAdvance(idMaze, candidates, current_pose,
                              next_directions);
  /* エラー防止のため来た方向を追加 */
  next_directions.next_direction_candidates.push_back(current_pose.d +
                                                      Direction::Back);
  /* 迷路をもとに戻す */
  std::reverse(wall_backup.begin(), wall_backup.end()); /* 重複対策 */
  for (const auto wr : wall_backup)
    idMaze.setWall(wr.getPosition(), wr.d, wr.b);
  /* 既知情報からではスタート区画を避けられない場合 */
  if (step_map.getStep(current_pose.p) == StepMap::STEP_MAX)
    calcNextDirectionsInAdvance(idMaze, candidates, current_pose,
                                next_directions);
  return next_directions.next_direction_candidates.empty() ? Error : Processing;
}

}  // namespace MazeLib
