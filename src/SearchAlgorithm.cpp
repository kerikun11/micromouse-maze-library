/**
 * @file SearchAlgorithm.cpp
 * @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
 * @author KERI (Github: kerikun11)
 * @url https://kerikeri.top/
 * @date 2017.11.05
 */
#include "SearchAlgorithm.h"

#include <algorithm>

namespace MazeLib {

/**
 * @brief 追加探索状態で探索を始める(ゴールを急がない)
 */
#define SEARCHING_ADDITIONALLY_AT_START 0

#define USE_OLD_ALGORITHM 1
#define USE_HYBRID_ALGORITHM 0

const char *SearchAlgorithm::stateString(const State s) {
  static const char *const str[] = {
      "Start                 ", "Searching for Goal    ",
      "Searching Additionally", "Backing to Start      ",
      "Reached Start         ", "Impossible            ",
      "Identifying Position  ", "Going to Goal         ",
  };
  return str[s];
}
/**
 * @brief 最短経路が導出されているか調べる関数
 */
bool SearchAlgorithm::isComplete() {
#if D_STAR_LITE_ENABLED
  shortestAlgorithm.Initialize();
#endif
  Vectors candidates;
  findShortestCandidates(candidates);
  return candidates.empty();
}
void SearchAlgorithm::positionIdentifyingInit(Vector *pVector, Dir *pDir) {
  idOffset = Vector(MAZE_SIZE / 2, MAZE_SIZE / 2);
  *pVector = idOffset;
  *pDir = Dir::East;
  idMaze.reset(false); /*< reset without setting start cell */
}
bool SearchAlgorithm::updateWall(const State state, const Vector v, const Dir d,
                                 const bool left, const bool front,
                                 const bool right, const bool back) {
  bool result = true;
  result = result & updateWall(state, v, d + Dir::Left, left);   // left wall
  result = result & updateWall(state, v, d + Dir::Front, front); // front wall
  result = result & updateWall(state, v, d + Dir::Right, right); // right wall
  result = result & updateWall(state, v, d + Dir::Back, back);   // back wall
  return result;
}
bool SearchAlgorithm::updateWall(const State state, const Vector v, const Dir d,
                                 const bool b) {
  bool result;
  if (state == IDENTIFYING_POSITION)
    result = idMaze.updateWall(v, d, b);
  else
    result = maze.updateWall(v, d, b);
#if D_STAR_LITE_ENABLED
  const bool known_only = false;
  const bool diag_enabled = true;
  shortestAlgorithm.UpdateChangedEdge(known_only, diag_enabled);
#endif
  return result;
}
void SearchAlgorithm::resetLastWall(const State state, const int num) {
  if (state == IDENTIFYING_POSITION)
    return idMaze.resetLastWall(num);
  return maze.resetLastWall(num);
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirs(
    State &state, Vector &curVec, Dir &curDir, Dirs &nextDirs,
    Dirs &nextDirCandidates, bool &isPositionIdentifying,
    bool &isForceBackToStart, bool &isForceGoingToGoal, int &matchCount) {
  state = START;
  Result result;
  /* check if in goal */
  if (!isPositionIdentifying && isForceGoingToGoal) {
    const auto goals = maze.getGoals();
    const auto it =
        std::find_if(goals.cbegin(), goals.cend(),
                     [curVec](const Vector nv) { return curVec == nv; });
    if (it != goals.end())
      isForceGoingToGoal = false;
  }
  /* position identification */
  if (isPositionIdentifying) {
    state = IDENTIFYING_POSITION;
    result = calcNextDirsPositionIdentification(curVec, curDir, nextDirs,
                                                nextDirCandidates, matchCount);
    switch (result) {
    case SearchAlgorithm::Processing:
      return result;
    case SearchAlgorithm::Reached:
      isPositionIdentifying = false;
      break; /*< go to next state */
    case SearchAlgorithm::Error:
      return result;
    }
  }
  /* search for goal */
  if (!SEARCHING_ADDITIONALLY_AT_START) {
    state = SEARCHING_FOR_GOAL;
    result =
        calcNextDirsSearchForGoal(curVec, curDir, nextDirs, nextDirCandidates);
    switch (result) {
    case SearchAlgorithm::Processing:
      return result;
    case SearchAlgorithm::Reached:
      break; /*< go to next state */
    case SearchAlgorithm::Error:
      return result;
    }
  }
  /* search additionally */
  if (!isForceBackToStart) {
    state = SEARCHING_ADDITIONALLY;
    result = calcNextDirsSearchAdditionally(curVec, curDir, nextDirs,
                                            nextDirCandidates);
    switch (result) {
    case SearchAlgorithm::Processing:
      return result;
    case SearchAlgorithm::Reached:
      break; /*< go to next state */
    case SearchAlgorithm::Error:
      return result;
    }
  }
  /* force going to goal */
  if (isForceGoingToGoal) {
    state = GOING_TO_GOAL;
    result =
        calcNextDirsGoingToGoal(curVec, curDir, nextDirs, nextDirCandidates);
    switch (result) {
    case SearchAlgorithm::Processing:
      return result;
    case SearchAlgorithm::Reached:
      isForceGoingToGoal = false;
      break; /*< go to next state */
    case SearchAlgorithm::Error:
      return result;
    }
  }
  /* backing to start */
  state = BACKING_TO_START;
  result =
      calcNextDirsBackingToStart(curVec, curDir, nextDirs, nextDirCandidates);
  switch (result) {
  case SearchAlgorithm::Processing:
    return result;
  case SearchAlgorithm::Reached:
    isForceBackToStart = false;
    break; /*< go to next state */
  case SearchAlgorithm::Error:
    return result;
  }
  /* reached start */
  state = REACHED_START;
  return result;
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
  /* find a direction it can go in nextDirCandidates */
  const auto it =
      std::find_if(nextDirCandidates.cbegin(), nextDirCandidates.cend(),
                   [&](const Dir dir) { return maze.canGo(v, dir); });
  if (it == nextDirCandidates.cend())
    return false; /*< no answer */
  nextDir = *it;
  return true;
}
bool SearchAlgorithm::calcShortestDirs(Dirs &shortestDirs,
                                       const bool diag_enabled) {
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
    else if (diag_enabled && rel_dir == Dir::Right)
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
#else
  /* new algorithm*/
  Indexes path;
  const bool known_only = true;
#if D_STAR_LITE_ENABLED
  shortestAlgorithm.Initialize();
  if (!shortestAlgorithm.ComputeShortestPath(known_only, diag_enabled))
    return false; /* 失敗 */
  if (!shortestAlgorithm.FollowShortestPath(path, known_only, diag_enabled))
    return false; /* 失敗 */
#else
  if (!shortestAlgorithm.calcShortestPath(path, known_only, diag_enabled))
    return false; /* 失敗 */
#endif
  shortestDirs = ShortestAlgorithm::indexes2dirs(path, diag_enabled);
  auto v = maze.getStart();
  for (const auto d : shortestDirs)
    v = v.next(d);
  if (shortestDirs.size() < 2)
    return true;
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
#if USE_OLD_ALGORITHM
  candidates.clear();
  /* no diag */
  {
    Dirs shortest_dirs;
    if (!stepMap.calcShortestDirs(maze, shortest_dirs, false))
      return false; /*< 失敗 */
    auto i = maze.getStart();
    for (const auto d : shortest_dirs) {
      if (!maze.isKnown(i, d))
        candidates.push_back(i);
      i = i.next(d);
    }
  }
  /* diag */
  {
    Dirs shortest_dirs;
    if (!stepMapWall.calcShortestDirs(maze, shortest_dirs, false))
      return false; /*< 失敗 */
    auto i = WallIndex(0, 0, 1);
    for (const auto d : shortest_dirs) {
      i = i.next(d);
      if (!maze.isKnown(i)) {
        candidates.push_back(i.getVector());
        candidates.push_back(i.getVector().next(i.getDir()));
      }
    }
  }
#if USE_HYBRID_ALGORITHM
  if (!candidates.empty())
#endif
    return true; /*< 成功 */
#endif
  /* 新アルゴリズム */
  candidates.clear();
#if D_STAR_LITE_ENABLED
  for (const auto diag_enabled : {true}) {
#else
  for (const auto diag_enabled : {true, false}) {
#endif
    Indexes path;
    const bool known_only = false;
#if D_STAR_LITE_ENABLED
    if (!shortestAlgorithm.ComputeShortestPath(known_only, diag_enabled))
      return false; /*< 失敗 */
    if (!shortestAlgorithm.FollowShortestPath(path, known_only, diag_enabled))
      return false; /*< 失敗 */
#else
    if (!shortestAlgorithm.calcShortestPath(path, known_only, diag_enabled))
      return false; /*< 失敗 */
#endif
    const auto dirs = ShortestAlgorithm::indexes2dirs(path, diag_enabled);
    auto v = maze.getStart();
    for (const auto d : dirs) {
      v = v.next(d);
      if (maze.unknownCount(v))
        candidates.push_back(v);
    }
  }
  return true; /*< 成功 */
}
int SearchAlgorithm::countIdentityCandidates(const WallLogs &idWallLogs,
                                             VecDir &ans) const {
  const int many = 1000;
  const int min_size = 12;
  const int min_diff = 6; /*< 許容食い違い壁数 */
  /* ある程度既知壁になるまで一致判定をしない */
  if (idWallLogs.size() < min_size)
    return many;
  /* min max */
  const int8_t max_x = maze.getMaxX() + 1;
  const int8_t max_y = maze.getMaxY() + 1;
  /* パターンマッチング開始 */
  int cnt = 0;
  for (int8_t x = 0; x < max_x; ++x)
    for (int8_t y = 0; y < max_y; ++y) {
      const auto offset_v = Vector(x, y);
      for (const auto offset_d : Dir::ENWS()) {
        int unknown = 0; /*< 未知壁数 */
        int diffs = 0;   /*< 既知壁との食い違い数を数える */
        for (const auto wl : idWallLogs) {
          const auto maze_v =
              (Vector(wl) - idOffset).rotate(offset_d) + offset_v;
          const auto maze_d = wl.d + offset_d;
          if (maze_v.isOutsideofField()) {
            diffs = many;
            break;
          }
          if (maze.isKnown(maze_v, maze_d) &&
              maze.isWall(maze_v, maze_d) != wl.b)
            ++diffs;
          if (!maze.isKnown(maze_v, maze_d))
            ++unknown;
          /* 打ち切り条件 */
          if (diffs > min_diff)
            break;
        }
        /* 非一致条件，要パラメータチューニング */
        if (diffs > min_diff || unknown * 5 > (int)idWallLogs.size() * 4)
          continue;
        /* 一致 */
        ans.first = offset_v;
        ans.second = offset_d;
        ++cnt;
        /* 打ち切り */
        if (cnt > 1)
          return many;
      }
    }
  return cnt;
}
const Dirs
SearchAlgorithm::findMatchDirCandidates(const Vector cur_v,
                                        const Vector target_v) const {
  Dirs result_dirs;
  for (const auto offset_d : Dir::ENWS()) {
    const auto offset_v = target_v - (cur_v - idOffset).rotate(offset_d);
    int diffs = 0; /*< 既知壁との食い違い数を数える */
    for (const auto wl : idMaze.getWallLogs()) {
      const auto maze_v = (Vector(wl) - idOffset).rotate(offset_d) + offset_v;
      const auto maze_d = wl.d + offset_d;
      if (maze_v.isOutsideofField()) {
        diffs = 9999;
        break;
      }
      if (maze.isKnown(maze_v, maze_d) && maze.isWall(maze_v, maze_d) != wl.b)
        ++diffs;
      /* 打ち切り */
      if (diffs > 0)
        break;
    }
    /* 非一致条件 */
    if (diffs > 0)
      continue;
    /* 一致 */
    result_dirs.push_back(Dir::South - offset_d);
  }
  // std::cout << cv << " Dirs: ";
  // for (auto d : result_dirs)
  //   std::cout << d << " ";
  // std::cout << "        " << std::endl;
  return result_dirs;
}
SearchAlgorithm::Result
SearchAlgorithm::calcNextDirsSearchForGoal(const Vector cv, const Dir cd,
                                           Dirs &nextDirsKnown,
                                           Dirs &nextDirCandidates) {
  Vectors candidates;
  for (const auto v : maze.getGoals())
    if (maze.unknownCount(v))
      candidates.push_back(v); /*< ゴール区画の未知区画を洗い出す */
  if (candidates.empty())
    return Reached;
  stepMap.calcNextDirs(maze, candidates, cv, cd, nextDirsKnown,
                       nextDirCandidates);
  return nextDirCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result
SearchAlgorithm::calcNextDirsSearchAdditionally(const Vector cv, const Dir cd,
                                                Dirs &nextDirsKnown,
                                                Dirs &nextDirCandidates) {
  Vectors candidates;
  findShortestCandidates(candidates); /*< 最短になりうる区画の洗い出し */
  if (candidates.empty())
    return Reached;
  stepMap.calcNextDirs(maze, candidates, cv, cd, nextDirsKnown,
                       nextDirCandidates);
  return nextDirCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result
SearchAlgorithm::calcNextDirsBackingToStart(const Vector cv, const Dir cd,
                                            Dirs &nextDirsKnown,
                                            Dirs &nextDirCandidates) {
  const auto v = stepMap.calcNextDirs(maze, {maze.getStart()}, cv, cd,
                                      nextDirsKnown, nextDirCandidates);
  if (v == maze.getStart())
    return Reached;
  return nextDirCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result
SearchAlgorithm::calcNextDirsGoingToGoal(const Vector cv, const Dir cd,
                                         Dirs &nextDirsKnown,
                                         Dirs &nextDirCandidates) {
  const auto &goals = maze.getGoals();
  stepMap.calcNextDirs(maze, goals, cv, cd, nextDirsKnown, nextDirCandidates);
  const auto nv = cv.next(nextDirCandidates[0] + Dir::Back);
  const auto it = std::find_if(goals.cbegin(), goals.cend(),
                               [nv](const auto v) { return nv == v; });
  if (it != goals.cend())
    return Reached;
  return nextDirCandidates.empty() ? Error : Processing;
}
SearchAlgorithm::Result SearchAlgorithm::calcNextDirsPositionIdentification(
    Vector &cv, Dir &cd, Dirs &nextDirsKnown, Dirs &nextDirCandidates,
    int &matchCount) {
  /* オフセットを調整する */
  if (!idMaze.getWallLogs().empty()) {
    const int8_t min_x = idMaze.getMinX();
    const int8_t min_y = idMaze.getMinY();
    const int8_t max_x = idMaze.getMaxX();
    const int8_t max_y = idMaze.getMaxY();
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
  VecDir ans;
  const int cnt = countIdentityCandidates(idMaze.getWallLogs(), ans);
  matchCount = cnt;
  if (cnt == 1) {
    cv = (cv - idOffset).rotate(ans.second) + ans.first;
    cd = cd + ans.second;
    return Reached;
  } else if (cnt == 0) {
    return Error;
  }
  /* min max */
  int8_t min_x = std::max(idMaze.getMinX() - 1, 0);
  int8_t min_y = std::max(idMaze.getMinY() - 1, 0);
  int8_t max_x = std::min(idMaze.getMaxX() + 2, MAZE_SIZE);
  int8_t max_y = std::min(idMaze.getMaxY() + 2, MAZE_SIZE);
  /* modify idMaze */
  WallLogs tmp;
  /* make candidates */
  Vectors candidates;
  if (idMaze.getWallLogs().empty())
    candidates.push_back(Vector(MAZE_SIZE / 2, MAZE_SIZE / 2));
  if (candidates.empty())
    for (int8_t x = min_x; x < max_x; ++x)
      for (int8_t y = min_y; y < max_y; ++y) {
        const auto v = Vector(x, y);
        /* スタート区画を避ける */
        const auto forbidden = findMatchDirCandidates(v, Vector(0, 1));
        for (const auto d : forbidden) {
          tmp.push_back(WallLog(v, d, idMaze.isWall(v, d)));
          idMaze.setWall(v, d, true);
        }
        if (idMaze.unknownCount(v))
          candidates.push_back(v);
      }
  /* 現在地の背後は壁なしにしないと移動できなくなることがある */
  tmp.push_back(WallLog(cv, cd + Dir::Back, idMaze.isWall(cv, cd + Dir::Back)));
  idMaze.setWall(cv, cd + Dir::Back, false);
  /* スタート区画を避けて導出 */
  stepMap.calcNextDirs(idMaze, candidates, cv, cd, nextDirsKnown,
                       nextDirCandidates);
  /* restore idMaze */
  std::reverse(tmp.begin(), tmp.end());
  for (const auto wl : tmp)
    idMaze.setWall(Vector(wl), wl.d, wl.b);
  /* 既知情報からではスタート区画が避けられない場合は普通に導出 */
  if (nextDirCandidates.empty())
    stepMap.calcNextDirs(idMaze, candidates, cv, cd, nextDirsKnown,
                         nextDirCandidates);
  /* end */
  return nextDirCandidates.empty() ? Error : Processing;
}

} // namespace MazeLib
