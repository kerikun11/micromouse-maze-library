/**
 *  @file SearchAlgorithm.cpp
 *  @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
 *  @author KERI (Github: kerikun11)
 *  @url https://kerikeri.top/
 *  @date 2017.11.05
 */
#include "SearchAlgorithm.h"

#include <algorithm>

namespace MazeLib {
/** @def SEARCHING_ADDITIALLY_AT_START
 *  @brief 追加探索状態で探索を始める(ゴールを急がない)
 */
#define SEARCHING_ADDITIALLY_AT_START 0

const char *SearchAlgorithm::stateString(const enum State s) {
  static const char *str[] = {
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
  Vectors candidates;
  findShortestCandidates(candidates);
  return candidates.empty();
}
void SearchAlgorithm::positionIdentifyingInit() {
  idStartVector = Vector(MAZE_SIZE / 2, MAZE_SIZE / 2);
  idMaze.reset(false);
}
bool SearchAlgorithm::updateWall(const State &state, const Vector &v,
                                 const Dir &d, const bool left,
                                 const bool front, const bool right,
                                 const bool back) {
  bool result = true;
  result = result & updateWall(state, v, d + 1, left);  // left wall
  result = result & updateWall(state, v, d + 0, front); // front wall
  result = result & updateWall(state, v, d - 1, right); // right wall
  result = result & updateWall(state, v, d + 2, back);  // back wall
  return result;
}
bool SearchAlgorithm::updateWall(const State &state, const Vector &v,
                                 const Dir &d, const bool &b) {
  if (state == IDENTIFYING_POSITION)
    return idMaze.updateWall(v, d, b);
  return maze.updateWall(v, d, b);
}
bool SearchAlgorithm::resetLastWall(const State &state, const int num) {
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
  if (!SEARCHING_ADDITIALLY_AT_START) {
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
bool SearchAlgorithm::findNextDir(const Maze &maze, const Vector v, const Dir d,
                                  const Dirs &nextDirCandidates,
                                  Dir &nextDir) const {
  // 候補の中で行ける方向を探す
  const auto it =
      std::find_if(nextDirCandidates.begin(), nextDirCandidates.end(),
                   [&](const Dir &dir) { return maze.canGo(v, dir); });
  if (it == nextDirCandidates.end())
    return false;
  nextDir = *it;
  return true;
}
bool SearchAlgorithm::calcShortestDirs(Dirs &shortestDirs,
                                       const bool diagonal) {
  stepMap.update(maze, maze.getGoals(), true, diagonal);
  // update(maze, maze.getGoals(), false, diagonal); //< for debug
  shortestDirs.clear();
  auto v = maze.getStart();
  Dir dir = Dir::North;
  auto prev_dir = dir;
  while (1) {
    step_t min_step = MAZE_STEP_MAX;
    prev_dir = dir;
    for (const auto d : Dir::All()) {
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
  // ゴール区画を行けるところまで直進する
  bool loop = true;
  while (loop) {
    loop = false;
    Dirs dirs;
    switch (Dir(dir - prev_dir)) {
    case Dir::Left:
      dirs = {Dir(dir + Dir::Right), dir};
      break;
    case Dir::Right:
      dirs = {Dir(dir + Dir::Left), dir};
      break;
    case Dir::Front:
    default:
      dirs = {dir};
      break;
    }
    if (!diagonal)
      dirs = {dir};
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
}
void SearchAlgorithm::printMap(const State state, const Vector vec,
                               const Dir dir) const {
  if (state == IDENTIFYING_POSITION)
    stepMap.print(idMaze, vec, dir);
  else
    stepMap.print(maze, vec, dir);
}

bool SearchAlgorithm::findShortestCandidates(Vectors &candidates) {
  candidates.clear();
  // 斜めありなしの双方の最短経路上を候補とする
  for (const bool diagonal : {true, false}) {
    stepMap.update(maze, maze.getGoals(), false, diagonal);
    auto v = maze.getStart();
    Dir dir = Dir::North;
    auto prev_dir = dir;
    while (1) {
      step_t min_step = MAZE_STEP_MAX;
      // const auto& dirs = dir.ordered(prev_dir);
      // prev_dir = dir;
      // for(const auto& d: dirs){
      for (const auto d : Dir::All()) {
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
    // ゴール区画を行けるところまで直進する
    bool loop = true;
    while (loop) {
      loop = false;
      Dirs dirs;
      switch (Dir(dir - prev_dir)) {
      case Dir::Left:
        dirs = {Dir(dir + Dir::Right), dir};
        break;
      case Dir::Right:
        dirs = {Dir(dir + Dir::Left), dir};
        break;
      case Dir::Front:
      default:
        dirs = {dir};
        break;
      }
      if (!diagonal)
        dirs = {dir};
      for (const auto &d : dirs) {
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
  return true; //< 成功
}
int SearchAlgorithm::countIdentityCandidates(
    const WallLogs idWallLogs, std::pair<Vector, Dir> &ans) const {
  int8_t max_x = 0;
  int8_t max_y = 0;
  for (const auto wl : getMaze().getWallLogs()) {
    if (wl.x >= MAZE_SIZE - 1 || wl.y >= MAZE_SIZE - 1)
      continue;
    max_x = std::max(max_x, wl.x);
    max_y = std::max(max_y, wl.y);
  }
  const int many = 1000;
  const int min_size = 10;
  if (idWallLogs.size() < min_size)
    return many;
  int cnt = 0;
  for (int x = -MAZE_SIZE / 2; x < -MAZE_SIZE / 2 + max_x; x++)
    for (int y = -MAZE_SIZE / 2; y < -MAZE_SIZE / 2 + max_y; y++)
      for (auto offset_d : Dir::All()) {
        Vector offset = Vector(x, y);
        int diffs = 0;
        int unknown = 0;
        for (auto wl : idWallLogs) {
          auto maze_v = Vector(wl).rotate(offset_d, idStartVector) + offset;
          Dir maze_d = wl.d + offset_d;
          if (maze.isKnown(maze_v, maze_d) &&
              maze.isWall(maze_v, maze_d) != wl.b)
            diffs++;
          if (!maze.isKnown(maze_v, maze_d))
            unknown++;
          if (diffs > MAZE_SIZE * 2)
            break;
        }
        // int size = idWallLogs.size();
        // int known = size - unknown;
        // int matchs = known - diffs;
        if (diffs < 5 && unknown < MAZE_SIZE) {
          ans.first = offset;
          ans.second = offset_d;
          cnt++;
        }
      }
  return cnt;
}
enum SearchAlgorithm::Status
SearchAlgorithm::calcNextDirsSearchForGoal(const Vector &cv, const Dir &cd,
                                           Dirs &nextDirsKnown,
                                           Dirs &nextDirCandidates) {
  Vectors candidates;
  for (auto v : maze.getGoals())
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
  auto it = std::find_if(goals.begin(), goals.end(),
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
  std::pair<Vector, Dir> ans;
  int cnt = countIdentityCandidates(idMaze.getWallLogs(), ans);
  matchCount = cnt;
  if (cnt == 1) {
    printf("Result: (%3d,%3d,%3c)\n", ans.first.x, ans.first.y,
           ">^<v"[ans.second]);
    cv = cv.rotate(ans.second, idStartVector) + ans.first;
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
