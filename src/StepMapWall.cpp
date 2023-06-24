/**
 * @file StepMapWall.cpp
 * @brief 壁ベースのステップマップを表現するクラス
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-08-17
 * @copyright Copyright 2019 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include "MazeLib/StepMapWall.h"

#include <algorithm> /*< for std::find */
#include <cmath>     /*< for std::sqrt */
#include <iomanip>   /*< for std::setw */
#include <queue>     /*< for std::priority_queue */

namespace MazeLib {

const WallIndex StepMapWall::START_WALL_INDEX = WallIndex(0, 0, 1);

void StepMapWall::print(const Maze& maze, const WallIndexes& indexes,
                        const bool showFullStep, std::ostream& os) const {
  const int mazeSize = MAZE_SIZE;
  const auto find = [&](const WallIndex& i) {
    return std::find(indexes.cbegin(), indexes.cend(), i) != indexes.cend();
  };
  step_t maxStep = 0;
  for (const auto step : stepMap)
    if (step != STEP_MAX) maxStep = std::max(maxStep, step);
  const bool simple = showFullStep || (maxStep < 999);
  const step_t scaler =
      stepTableDiag[stepTableSize - 1] - stepTableDiag[stepTableSize - 2];
  /* start to draw maze */
  for (int8_t y = mazeSize - 1; y >= -1; --y) {
    /* Horizontal Wall Line */
    for (int8_t x = 0; x < mazeSize; ++x) {
      /* Pillar */
      os << '+';
      /* Horizontal Wall */
      const auto w = maze.isWall(x, y, Direction::North);
      const auto k = maze.isKnown(x, y, Direction::North);
      const auto i = WallIndex(Position(x, y), Direction::North);
      auto s = getStep(i);
      s = std::min(simple ? s : s / scaler, showFullStep ? 99999 : 999);
      const auto f = find(i);
      if (w)
        os << (showFullStep ? "-----" : "---");
      else
        os << (f ? "\e[43m" C_BL : (s == 0 ? C_YE : (k ? C_BL : C_RE)))
           << std::setw(showFullStep ? 5 : 3) << s << C_NO;
    }
    os << '+' << std::endl;
    /* Vertical Wall Line */
    if (y != -1) {
      os << (showFullStep ? "|  " : "| ");
      for (int8_t x = 0; x < mazeSize; ++x) {
        /* Cell */
        os << ' ';
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::East);
        const auto k = maze.isKnown(x, y, Direction::East);
        const auto i = WallIndex(Position(x, y), Direction::East);
        auto s = getStep(i);
        s = std::min(simple ? s : s / scaler, showFullStep ? 99999 : 999);
        const auto f = find(i);
        if (w)
          os << (showFullStep ? "  |  " : " | ");
        else
          os << (f ? "\e[43m" C_BL : (s == 0 ? C_YE : (k ? C_BL : C_RE)))
             << std::setw(showFullStep ? 5 : 3) << s << C_NO;
      }
      os << std::endl;
    }
  }
}
void StepMapWall::print(const Maze& maze, const Directions& shortestDirections,
                        const WallIndex& start, const bool showFullStep,
                        std::ostream& os) const {
  auto i = start;
  WallIndexes shortestIndexes;
  shortestIndexes.reserve(shortestDirections.size() + 1);
  shortestIndexes.push_back(i);
  for (const auto d : shortestDirections) {
    i = i.next(d);
    shortestIndexes.push_back(i);
  }
  print(maze, shortestIndexes, showFullStep, os);
}
void StepMapWall::printPath(const Maze& maze, const WallIndexes& indexes,
                            std::ostream& os) const {
  const auto exists = [&](const WallIndex& i) {
    return std::find(indexes.cbegin(), indexes.cend(), i) != indexes.cend();
  };
  for (int8_t y = MAZE_SIZE - 1; y >= -1; --y) {
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      /* Pillar */
      os << '+';
      /* Horizontal Wall */
      const auto w = maze.isWall(x, y, Direction::North);
      const auto k = maze.isKnown(x, y, Direction::North);
      if (exists(WallIndex(x, y, 1)))
        os << C_YE " X " C_NO;
      else
        os << (k ? (w ? "---" : "   ") : (C_RE " . " C_NO));
    }
    os << '+' << std::endl;
    if (y != -1) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        /* Cell */
        os << "   ";
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::East);
        const auto k = maze.isKnown(x, y, Direction::East);
        if (exists(WallIndex(x, y, 0)))
          os << C_YE "X" C_NO;
        else
          os << (k ? (w ? "|" : " ") : (C_RE "." C_NO));
      }
      os << std::endl;
    }
  }
}
void StepMapWall::printPath(const Maze& maze,
                            const Directions& shortestDirections,
                            const WallIndex& start, std::ostream& os) const {
  auto i = start;
  WallIndexes shortestIndexes;
  shortestIndexes.reserve(shortestDirections.size() + 1);
  shortestIndexes.push_back(i);
  for (const auto d : shortestDirections) {
    i = i.next(d);
    shortestIndexes.push_back(i);
  }
  printPath(maze, shortestIndexes, os);
}
void StepMapWall::update(const Maze& maze, const WallIndexes& dest,
                         const bool knownOnly, const bool simple) {
  MAZE_DEBUG_PROFILING_START(0)
  /* 全区画のステップを最大値に設定 */
  const auto step = STEP_MAX;
  stepMap.fill(step);
  /* ステップの更新予約のキュー */
#define STEP_MAP_USE_PRIORITY_QUEUE 1
#if STEP_MAP_USE_PRIORITY_QUEUE
  struct Element {
    WallIndex p;
    step_t s;
    bool operator<(const Element& e) const { return s > e.s; }
  };
  std::priority_queue<Element> q;
#else
  std::queue<WallIndex> q;
#endif
  /* destのステップを0とする */
  for (const auto i : dest)
    if (i.isInsideOfField() && !maze.isWall(i))
#if STEP_MAP_USE_PRIORITY_QUEUE
      stepMap[i.getIndex()] = 0, q.push({i, 0});
#else
      stepMap[i.getIndex()] = 0, q.push(i);
#endif
  /* ステップの更新がなくなるまで更新処理 */
  while (!q.empty()) {
#if MAZE_DEBUG_PROFILING
    queueSizeMax = std::max(queueSizeMax, q.size());
#endif
    /* 注目する壁を取得 */
#if STEP_MAP_USE_PRIORITY_QUEUE
    const auto focus = q.top().p;
    const auto focus_step_q = q.top().s;
#else
    const auto focus = q.front();
#endif
    q.pop();
    const auto focus_step = stepMap[focus.getIndex()];
#if STEP_MAP_USE_PRIORITY_QUEUE
    /* 枝刈り */
    if (focus_step < focus_step_q) continue;
#endif
    /* 周辺を走査 */
    for (const auto d : focus.getNextDirection6()) {
      const auto& stepTable = (d.isAlong() ? stepTableAlong : stepTableDiag);
      /* 直線で行けるところまで更新する */
      auto next = focus;
      for (int8_t i = 1;; ++i) {
        next = next.next(d); /*< 移動 */
        /* 壁あり or 既知壁のみで未知壁 ならば次へ */
        if (maze.isWall(next) || (knownOnly && !maze.isKnown(next))) break;
        /* 直線加速を考慮したステップを算出 */
        const step_t next_step = focus_step + (simple ? i : stepTable[i]);
        const auto next_index = next.getIndex();
        if (stepMap[next_index] <= next_step) break; /*< 更新の必要がない */
        stepMap[next_index] = next_step;             /*< 更新 */
        /* 再帰的に更新するためにキューにプッシュ */
#if STEP_MAP_USE_PRIORITY_QUEUE
        q.push({next, next_step});
#else
        q.push(next);
#endif
      }
    }
  }
  MAZE_DEBUG_PROFILING_END(0)
}
Directions StepMapWall::calcShortestDirections(const Maze& maze,
                                               const WallIndex& start,
                                               const WallIndexes& dest,
                                               const bool knownOnly,
                                               const bool simple) {
  /* ステップマップを更新 */
  update(maze, dest, knownOnly, simple);
  WallIndex end;
  const auto shortestDirections =
      getStepDownDirections(maze, start, end, knownOnly, simple, false);
  /* ゴール判定 */
  return stepMap[end.getIndex()] == 0 ? shortestDirections : Directions{};
}
Directions StepMapWall::getStepDownDirections(
    const Maze& maze, const WallIndex& start, WallIndex& end,
    const bool knownOnly, const bool simple, const bool breakUnknown) const {
#if 0
  /* 最短経路となるスタートからの方向列 */
  Directions shortestDirections;
  auto& focus = end;
  /* start から順にステップマップを下る */
  focus = start;
  /* 確認 */
  if (!start.isInsideOfField())
    return {};
  /* 周辺の走査; 未知壁の有無と最小ステップの方向を求める */
  while (1) {
    const auto focus_step = stepMap[focus.getIndex()];
    // MAZE_LOGI << focus << "\t" << focus_step << std::endl;
    /* 終了条件 */
    if (focus_step == 0)
      break;
    /* 周辺を走査 */
    auto min_p = focus;
    auto min_d = Direction::Max;
    for (const auto d : focus.getNextDirection6()) {
      const auto& stepTable =
          (d.isAlong() ? stepTableAlong : stepTableDiag);
      /* 直線で行けるところまで探す */
      auto next = focus;
      for (int8_t i = 1;; ++i) {
        next = next.next(d); /*< 移動 */
        /* 壁あり or 既知壁のみで未知壁 ならば次へ */
        if (maze.isWall(next) || (knownOnly && !maze.isKnown(next)))
          break;
        /* 直線加速を考慮したステップを算出 */
        const step_t next_step = focus_step - (simple ? i : stepTable[i]);
        /* エッジコストと一致する確認 */
        if (stepMap[next.getIndex()] == next_step) {
          min_p = next, min_d = d;
          goto loop_exit;
        }
      }
    }
  loop_exit:
    /* 現在地よりステップが大きかったらなんかおかしい */
    if (focus_step <= stepMap[min_p.getIndex()]) {
      MAZE_LOGE << "error" << std::endl;
      break;
    }
    /* 移動分を結果に追加 */
    while (focus != min_p) {
      /* breakUnknown のとき、未知壁を含むならば既知区間は終了 */
      if (breakUnknown && maze.isKnown(focus))
        return shortestDirections;
      focus = focus.next(min_d);
      shortestDirections.push_back(min_d);
    }
  }
  return shortestDirections;
#else
  MAZE_DEBUG_PROFILING_START(0)
  /* 壁ベースはたどり方に注意。135度ターンが鋭角にならないように1マスずつ。 */
  (void)simple;
  (void)breakUnknown;
  /* 最短経路となるスタートからの方向列 */
  Directions shortestDirections;
  /* start から順にステップマップを下る */
  end = start;
  /* 確認 */
  if (!start.isInsideOfField()) return {};
  while (1) {
    /* 周辺の走査; 未知壁の有無と、最小ステップの方向を求める */
    auto min_d = Direction::Max;
    auto min_step = STEP_MAX;
    /* 周辺を走査 */
    for (const auto d : end.getNextDirection6()) {
      auto next = end;     /*< 隣接 */
      next = next.next(d); /*< 移動 */
      /* 壁あり or 既知壁のみで未知壁 ならば次へ */
      if (maze.isWall(next) || (knownOnly && !maze.isKnown(next))) continue;
      /* min_step よりステップが小さければ更新 (同じなら更新しない) */
      const auto next_step = stepMap[next.getIndex()];
      if (min_step < next_step) continue;
      min_step = next_step;
      min_d = d;
    }
    /* 現在地のステップより大きかったらなんかおかしい */
    if (stepMap[end.getIndex()] <= min_step) break;
    end = end.next(min_d);                //< 位置を更新
    shortestDirections.push_back(min_d);  //< 既知区間移動
  }
  MAZE_DEBUG_PROFILING_END(0)
  return shortestDirections;
#endif
}
WallIndexes StepMapWall::convertDestinations(const Maze& maze,
                                             const Positions& positions) {
  WallIndexes dest;
  for (const auto p : positions)
    for (const auto d : Direction::Along4)
      if (!maze.isWall(p, d)) dest.push_back(WallIndex(p, d));
  return dest;
}
Direction StepMapWall::convertWallIndexDirection(const WallIndex& i,
                                                 const Direction d) {
  switch (d) {
    case Direction::East:
    case Direction::North:
    case Direction::West:
    case Direction::South:
      return d;
    case Direction::NorthEast:
      return i.z == 0 ? Direction::North : Direction::East;
    case Direction::SouthWest:
      return i.z == 0 ? Direction::South : Direction::West;
    case Direction::NorthWest:
      return i.z == 0 ? Direction::North : Direction::West;
    case Direction::SouthEast:
      return i.z == 0 ? Direction::South : Direction::East;
    default:
      MAZE_LOGE << "invalid direction" << std::endl;
      return Direction::Max;
  }
}
Directions StepMapWall::convertWallIndexDirectionsToPositionDirections(
    const Directions& src) {
  if (src.size() < 2) return {};
  Directions dirs;
  dirs.reserve(src.size() + 1);
  auto i = START_WALL_INDEX;
  dirs.push_back(Direction::North);  //< start cell
  for (const auto d : src) {
    dirs.push_back(convertWallIndexDirection(i, d));
    i = i.next(d);
  }
  return dirs;
}
void StepMapWall::appendStraightDirections(const Maze& maze,
                                           Directions& shortestDirections,
                                           const WallIndex& start) {
  auto i = start;
  for (const auto d : shortestDirections) i = i.next(d);
  if (shortestDirections.size()) {
    const auto d = shortestDirections.back();
    while (1) {
      i = i.next(d);
      if (maze.isWall(i)) break;
      shortestDirections.push_back(d);
    }
  }
}

/**
 * @brief 台形加速を考慮したコストを生成する関数
 *
 * @param i マスの数
 * @param am 最大加速度
 * @param vs 始点速度
 * @param vm 飽和速度
 * @param seg 1マスの長さ
 * @return StepMap::step_t コスト
 */
static StepMapWall::step_t calcStraightCost(const int i, const float am,
                                            const float vs, const float vm,
                                            const float seg) {
  const auto d = seg * i; /*< i 区画分の走行距離 */
  /* グラフの面積から時間を求める */
  const auto d_thr = (vm * vm - vs * vs) / am; /*< 最大速度に達する距離 */
  if (d < d_thr)
    return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 1000; /*< 三角加速 */
  else
    return (am * d + (vm - vs) * (vm - vs)) / (am * vm) * 1000; /*< 台形加速 */
}
void StepMapWall::calcStraightCostTable() {
  const float vs = 420.0f;    /*< 基本速度 [mm/s] */
  const float am_a = 4200.0f; /*< 最大加速度 [mm/s/s] */
  const float am_d = 3600.0f; /*< 最大加速度(斜め) [mm/s/s] */
  const float vm_a = 1500.0f; /*< 飽和速度 [mm/s] */
  const float vm_d = 1200.0f; /*< 飽和速度(斜め) [mm/s] */
  const float seg_a = 90.0f;  /*< 1区画の長さ [mm] */
  const float seg_d = 45.0f * std::sqrt(2.0f); /*< 1区画の長さ(斜め) [mm] */
  const float t_turn = 388.0f;              /*< FV90ターンの時間 [ms] */
  stepTableAlong[0] = stepTableDiag[0] = 0; /*< [0] は使用しない */
  for (int i = 1; i < stepTableSize; ++i) {
    /* V90があるので斜め側でターンのコストを考慮 */
    stepTableAlong[i] = calcStraightCost(i, am_a, vs, vm_a, seg_a);
    stepTableDiag[i] = t_turn + calcStraightCost(i - 1, am_d, vs, vm_d, seg_d);
  }
  /* コストの合計が 65,535 [ms] を超えないようにスケーリング */
  for (int i = 0; i < stepTableSize; ++i) {
    stepTableAlong[i] /= scalingFactor;
    stepTableDiag[i] /= scalingFactor;
#if 0
    MAZE_LOGI << "stepTableAlong[" << i << "]:\t" << stepTableAlong[i] << "\t"
              << "stepTableDiag[" << i << "]:\t" << stepTableDiag[i]
              << std::endl;
#endif
  }
}

};  // namespace MazeLib
