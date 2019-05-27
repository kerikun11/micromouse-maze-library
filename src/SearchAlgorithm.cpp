/**
 *  @file SearchAlgorithm.cpp
 *  @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
 *  @author KERI (Github: kerikun11)
 *  @url https://kerikeri.top/
 *  @date 2017.11.05
 */
#include "SearchAlgorithm.h"

#include "ShortestAlgorithm.h"

#include <algorithm>
#include <chrono>

namespace MazeLib {

/** @def SEARCHING_ADDITIONALLY_AT_START
 *  @brief 追加探索状態で探索を始める(ゴールを急がない)
 */
#define SEARCHING_ADDITIONALLY_AT_START 1

const char *SearchAlgorithm::stateString(const enum State s) {
  static const char *const str[] = {
      "Start                 ", "Searching for Goal    ",
      "Searching Additionally", "Backing to Start      ",
      "Reached Start         ", "Impossible            ",
      "Identifying Position  ", "Going to Goal         ",
  };
  return str[s];
}
/** @function isComplete
 *  @brief 最短経路が導出されているか調べる関数
 */
bool SearchAlgorithm::isComplete() {
  shortestAlgorithm.Initialize();
  Vectors candidates;
  findShortestCandidates(candidates);
  return candidates.empty();
}
void SearchAlgorithm::positionIdentifyingInit(Vector *pVector, Dir *pDir) {
  idOffset = Vector(MAZE_SIZE / 2, MAZE_SIZE / 2);
  *pVector = idOffset;
  *pDir = Dir::East;
  idMaze.reset(false);
}
bool SearchAlgorithm::updateWall(const State &state, const Vector &v,
                                 const Dir &d, const bool left,
                                 const bool front, const bool right,
                                 const bool back) {
  bool result = true;
  result = result & updateWall(state, v, d + Dir::Left, left); // left wall
  shortestAlgorithm.UpdateChangedEdge(false, true);
  result = result & updateWall(state, v, d + Dir::Front, front); // front wall
  shortestAlgorithm.UpdateChangedEdge(false, true);
  result = result & updateWall(state, v, d + Dir::Right, right); // right wall
  shortestAlgorithm.UpdateChangedEdge(false, true);
  result = result & updateWall(state, v, d + Dir::Back, back); // back wall
  shortestAlgorithm.UpdateChangedEdge(false, true);
  return result;
}
bool SearchAlgorithm::updateWall(const State &state, const Vector &v,
                                 const Dir &d, const bool &b) {
  if (state == IDENTIFYING_POSITION)
    return idMaze.updateWall(v, d, b);
  return maze.updateWall(v, d, b);
}
void SearchAlgorithm::resetLastWall(const State &state, const int num) {
  if (state == IDENTIFYING_POSITION)
    return idMaze.resetLastWall(num);
  return maze.resetLastWall(num);
}
enum SearchAlgorithm::Status SearchAlgorithm::calcNextDirs(
    State &state, Vector &curVec, Dir &curDir, Dirs &nextDirs,
    Dirs &nextDirCandidates, bool &isPositionIdentifying,
    bool &isForceBackToStart, bool &isForceGoingToGoal, int &matchCount) {
  state = START;
  SearchAlgorithm::Status status;
  if (isPositionIdentifying) {
    state = IDENTIFYING_POSITION;
    status = calcNextDirsPositionIdentification(curVec, curDir, nextDirs,
                                                nextDirCandidates, matchCount);
    switch (status) {
    case SearchAlgorithm::Processing:
      return status;
    case SearchAlgorithm::Reached:
      isPositionIdentifying = false;
      break;
    case SearchAlgorithm::Error:
      return status;
    }
  }
  if (!SEARCHING_ADDITIONALLY_AT_START) {
    state = SEARCHING_FOR_GOAL;
    status =
        calcNextDirsSearchForGoal(curVec, curDir, nextDirs, nextDirCandidates);
    switch (status) {
    case SearchAlgorithm::Processing:
      return status;
    case SearchAlgorithm::Reached:
      break;
    case SearchAlgorithm::Error:
      return status;
    }
  }
  if (!isForceBackToStart) {
    state = SEARCHING_ADDITIONALLY;
    status = calcNextDirsSearchAdditionally(curVec, curDir, nextDirs,
                                            nextDirCandidates);
    switch (status) {
    case SearchAlgorithm::Processing:
      return status;
    case SearchAlgorithm::Reached:
      break;
    case SearchAlgorithm::Error:
      return status;
    }
  }
  if (isForceGoingToGoal) {
    state = GOING_TO_GOAL;
    status =
        calcNextDirsGoingToGoal(curVec, curDir, nextDirs, nextDirCandidates);
    switch (status) {
    case SearchAlgorithm::Processing:
      return status;
    case SearchAlgorithm::Reached:
      isForceGoingToGoal = false;
      return SearchAlgorithm::Processing;
    case SearchAlgorithm::Error:
      return status;
    }
  }
  state = BACKING_TO_START;
  status =
      calcNextDirsBackingToStart(curVec, curDir, nextDirs, nextDirCandidates);
  switch (status) {
  case SearchAlgorithm::Processing:
    return status;
  case SearchAlgorithm::Reached:
    break;
  case SearchAlgorithm::Error:
    return status;
  }
  state = REACHED_START;
  return status;
}
bool SearchAlgorithm::findNextDir(const State state, const Vector v,
                                  const Dir d, const Dirs &nextDirCandidates,
                                  Dir &nextDir) const {
  return findNextDir(state == IDENTIFYING_POSITION ? idMaze : maze, v, d,
                     nextDirCandidates, nextDir);
}
bool SearchAlgorithm::findNextDir(const Maze &maze, const Vector v,
                                  const Dir d __attribute__((unused)),
                                  const Dirs &nextDirCandidates,
                                  Dir &nextDir) const {
  // 候補の中で行ける方向を探す
  const auto it =
      std::find_if(nextDirCandidates.cbegin(), nextDirCandidates.cend(),
                   [&](const Dir &dir) { return maze.canGo(v, dir); });
  if (it == nextDirCandidates.end())
    return false;
  nextDir = *it;
  return true;
}
bool SearchAlgorithm::calcShortestDirs(Dirs &shortestDirs,
                                       const bool diag_enabled) {
  /* new algorithm*/
  ShortestAlgorithm::Indexes path;
  shortestAlgorithm.Initialize();
  if (!shortestAlgorithm.ComputeShortestPath(true, diag_enabled))
    return false; /* 失敗 */
  if (!shortestAlgorithm.FollowShortestPath(path, true, diag_enabled))
    return false; /* 失敗 */
  shortestDirs = ShortestAlgorithm::indexes2dirs(path, diag_enabled);
  auto v = maze.getStart();
  for (const auto d : shortestDirs)
    v = v.next(d);
  auto prev_dir = shortestDirs[shortestDirs.size() - 1 - 1];
  auto dir = shortestDirs[shortestDirs.size() - 1];
  // ゴール区画を行けるところまで直進(斜め考慮)する
  bool loop = true;
  while (loop) {
    loop = false;
    // 斜めを考慮した進行方向を列挙する
    Dirs dirs;
    const auto rel_dir = Dir(dir - prev_dir);
    if (diag_enabled && rel_dir == Dir::Left)
      dirs = {Dir(dir + Dir::Right), dir};
    else if (diag_enabled && rel_dir == Dir::Right)
      dirs = {Dir(dir + Dir::Left), dir};
    else
      dirs = {dir};
    // 行ける方向に行く
    for (const auto d : dirs) {
      if (maze.canGo(v, d)) {
        shortestDirs.push_back(d);
        v = v.next(d);
        prev_dir = dir;
        dir = d;
        loop = true;
        break;
      }
    }
  }
  return true; /* 成功 */
#if 0
  /* old */
  stepMap.update(maze, maze.getGoals(), true, diag_enabled);
  shortestDirs.clear();
  auto v = maze.getStart();
  Dir dir = Dir::North;
  auto prev_dir = dir;
  while (1) {
    step_t min_step = MAZE_STEP_MAX;
    prev_dir = dir;
    for (const auto d : Dir::ENWS()) {
      if (!maze.canGo(v, d))
        continue;
      step_t next_step = stepMap.getStep(v.next(d));
      if (min_step > next_step) {
        min_step = next_step;
        dir = d;
      }
    }
    if (stepMap.getStep(v) <= min_step)
      return false; //< 失敗
    shortestDirs.push_back(dir);
    v = v.next(dir);
    if (stepMap.getStep(v) == 0)
      break; //< ゴール区画
  }
  // ゴール区画を行けるところまで直進(斜め考慮)する
  bool loop = true;
  while (loop) {
    loop = false;
    // 斜めを考慮した進行方向を列挙する
    Dirs dirs;
    const auto rel_dir = Dir(dir - prev_dir);
    if (diag_enabled && rel_dir == Dir::Left)
      dirs = {Dir(dir + Dir::Right), dir};
    else if (diagonal && rel_dir == Dir::Right)
      dirs = {Dir(dir + Dir::Left), dir};
    else
      dirs = {dir};
    // 行ける方向に行く
    for (const auto &d : dirs) {
      if (maze.canGo(v, d)) {
        shortestDirs.push_back(d);
        v = v.next(d);
        prev_dir = dir;
        dir = d;
        loop = true;
        break;
      }
    }
  }
  return true;
#endif
}

void SearchAlgorithm::printMap(const State state, const Vector vec,
                               const Dir dir) const {
  if (state == IDENTIFYING_POSITION)
    stepMap.print(idMaze, vec, dir);
  else
    stepMap.print(maze, vec, dir);
}

bool SearchAlgorithm::findShortestCandidates(Vectors &candidates) {
#if 0
  /* old */
  candidates.clear();
  // 斜めありなしの双方の最短経路上を候補とする
  for (const bool diagonal : {true, false}) {
    stepMap.update(maze, maze.getGoals(), false, diagonal);
    auto v = maze.getStart();
    Dir dir = Dir::North;
    auto prev_dir = dir;
    while (1) {
      step_t min_step = MAZE_STEP_MAX;
      // 周囲のマスの中で一番ステップの小さいマスに移動
      for (const auto d : Dir::ENWS()) {
        if (maze.isWall(v, d))
          continue;
        step_t next_step = stepMap.getStep(v.next(d));
        if (min_step > next_step) {
          min_step = next_step;
          dir = d;
        }
      }
      if (stepMap.getStep(v) <= min_step)
        return false; //< 失敗
      if (maze.unknownCount(v))
        candidates.push_back(v); //< 未知壁があれば候補に入れる
      v = v.next(dir);
      if (stepMap.getStep(v) == 0)
        break; //< ゴール区画
    }
    // ゴール区画を行けるところまで直進(斜め考慮)する
    bool loop = true;
    while (loop) {
      loop = false;
      // 斜めを考慮した進行方向を列挙する
      Dirs dirs;
      const auto rel_dir = Dir(dir - prev_dir);
      if (diagonal && rel_dir == Dir::Left)
        dirs = {Dir(dir + Dir::Right), dir};
      else if (diagonal && rel_dir == Dir::Right)
        dirs = {Dir(dir + Dir::Left), dir};
      else
        dirs = {dir};
      // 行ける方向に行く
      for (const auto d : dirs) {
        if (!maze.isWall(v, d)) {
          if (maze.unknownCount(v))
            candidates.push_back(v); //< 未知壁があれば候補に入れる
          v = v.next(d);
          prev_dir = dir;
          dir = d;
          loop = true;
          break;
        }
      }
    }
  }
  if (!candidates.empty())
    return true; //< 成功
#endif
  /* 新アルゴリズム */
  candidates.clear();
  // for (const auto diag_enabled : {true, false}) {
  for (const auto diag_enabled : {true}) {
    ShortestAlgorithm::Indexes path;
    // if (!shortestAlgorithm.calcShortestPath(path, false, diag_enabled))
    if (!shortestAlgorithm.ComputeShortestPath(false, diag_enabled))
      return false; /* 失敗 */
    if (!shortestAlgorithm.FollowShortestPath(path, false, diag_enabled))
      return false; /* 失敗 */
    const auto dirs = ShortestAlgorithm::indexes2dirs(path, diag_enabled);
    auto v = maze.getStart();
    for (const auto d : dirs) {
      v = v.next(d);
      if (maze.unknownCount(v))
        candidates.push_back(v);
    }
  }
  return true; /* 成功 */
}
int SearchAlgorithm::countIdentityCandidates(
    const WallLogs &idWallLogs, std::pair<Vector, Dir> &ans) const {
  const int many = 1000;
  const int min_size = 12;
  const int min_diff = 4;
  if (idWallLogs.size() < min_size)
    return many;
  int cnt = 0;
  for (int x = 0; x < MAZE_SIZE; ++x)
    for (int y = 0; y < MAZE_SIZE; ++y)
      for (const auto offset_d : Dir::ENWS()) {
        Vector offset = Vector(x, y);
        int diffs = 0;
        int unknown = 0;
        for (const auto wl : idWallLogs) {
          const auto maze_v = (Vector(wl) - idOffset).rotate(offset_d) + offset;
          const auto maze_d = wl.d + offset_d;
          if (!maze_v.isInsideOfField()) {
            diffs = many;
            break;
          }
          if (maze.isKnown(maze_v, maze_d) &&
              maze.isWall(maze_v, maze_d) != wl.b)
            diffs++;
          if (!maze.isKnown(maze_v, maze_d))
            unknown++;
          if (diffs > min_diff)
            break;
        }
        const int size = idWallLogs.size();
        const int known = size - unknown;
        // int matchs = known - diffs;
        if (diffs <= min_diff && known > unknown) {
          ans.first = offset;
          ans.second = offset_d;
          cnt++;
          if (cnt > 1)
            return many;
        }
      }
  return cnt;
}
enum SearchAlgorithm::Status
SearchAlgorithm::calcNextDirsSearchForGoal(const Vector &cv, const Dir &cd,
                                           Dirs &nextDirsKnown,
                                           Dirs &nextDirCandidates) {
  Vectors candidates;
  for (const auto v : maze.getGoals())
    if (maze.unknownCount(v))
      candidates.push_back(v); //< ゴール区画の未知区画を洗い出す
  if (candidates.empty())
    return Reached;
  stepMap.calcNextDirs(maze, candidates, cv, cd, nextDirsKnown,
                       nextDirCandidates);
  return nextDirCandidates.empty() ? Error : Processing;
}
enum SearchAlgorithm::Status
SearchAlgorithm::calcNextDirsSearchAdditionally(const Vector &cv, const Dir &cd,
                                                Dirs &nextDirsKnown,
                                                Dirs &nextDirCandidates) {
  Vectors candidates;
  findShortestCandidates(candidates); //< 最短になりうる区画の洗い出し
  if (candidates.empty())
    return Reached;
  stepMap.calcNextDirs(maze, candidates, cv, cd, nextDirsKnown,
                       nextDirCandidates);
  return nextDirCandidates.empty() ? Error : Processing;
}
enum SearchAlgorithm::Status
SearchAlgorithm::calcNextDirsBackingToStart(const Vector &cv, const Dir &cd,
                                            Dirs &nextDirsKnown,
                                            Dirs &nextDirCandidates) {
  const auto v = stepMap.calcNextDirs(maze, {maze.getStart()}, cv, cd,
                                      nextDirsKnown, nextDirCandidates);
  if (v == maze.getStart())
    return Reached;
  return nextDirCandidates.empty() ? Error : Processing;
}
enum SearchAlgorithm::Status
SearchAlgorithm::calcNextDirsGoingToGoal(const Vector &cv, const Dir &cd,
                                         Dirs &nextDirsKnown,
                                         Dirs &nextDirCandidates) {
  const auto goals = maze.getGoals();
  const auto v = stepMap.calcNextDirs(maze, goals, cv, cd, nextDirsKnown,
                                      nextDirCandidates);
  const auto it = std::find_if(goals.cbegin(), goals.cend(),
                               [v](const Vector nv) { return v == nv; });
  if (it != goals.end())
    return Reached;
  return nextDirCandidates.empty() ? Error : Processing;
}
enum SearchAlgorithm::Status
SearchAlgorithm::calcNextDirsPositionIdentification(Vector &cv, Dir &cd,
                                                    Dirs &nextDirsKnown,
                                                    Dirs &nextDirCandidates,
                                                    int &matchCount) {
  if (!idMaze.getWallLogs().empty()) {
    int8_t min_x = MAZE_SIZE - 1;
    int8_t min_y = MAZE_SIZE - 1;
    int8_t max_x = 0;
    int8_t max_y = 0;
    for (const auto wl : idMaze.getWallLogs()) {
      min_x = std::min((int8_t)wl.x, min_x);
      min_y = std::min((int8_t)wl.y, min_y);
      max_x = std::max((int8_t)wl.x, max_x);
      max_y = std::max((int8_t)wl.y, max_y);
    }
    const auto offset_new =
        idOffset + Vector((MAZE_SIZE - max_x - min_x - 1) / 2,
                          (MAZE_SIZE - max_y - min_y - 1) / 2);
    const auto offset_diff = offset_new - idOffset;
    idOffset = offset_new;
    cv = cv + offset_diff;
    WallLogs tmp = idMaze.getWallLogs();
    idMaze.reset(false);
    for (const auto wl : tmp)
      idMaze.updateWall(Vector(wl) + offset_diff, wl.d, wl.b);
  }
  /* 自己位置同定 */
  std::pair<Vector, Dir> ans;
  const int cnt = countIdentityCandidates(idMaze.getWallLogs(), ans);
  matchCount = cnt;
  if (cnt == 1) {
    cv = (cv - idOffset).rotate(ans.second) + ans.first;
    cd = cd + ans.second;
    return Reached;
  } else if (cnt == 0) {
    return Error;
  }
  Vectors candidates;
  for (int8_t x = 0; x < MAZE_SIZE; ++x)
    for (int8_t y = 0; y < MAZE_SIZE; ++y)
      if (idMaze.unknownCount(Vector(x, y)))
        candidates.push_back(Vector(x, y));
  stepMap.calcNextDirs(idMaze, candidates, cv, cd, nextDirsKnown,
                       nextDirCandidates);
  return nextDirCandidates.empty() ? Error : Processing;
}

} // namespace MazeLib
