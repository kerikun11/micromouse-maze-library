/**
 * @file StepMapWall.cpp
 * @brief 壁ベースのステップマップを表現するクラス
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-08-17
 * @copyright Copyright 2019 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include "MazeLib/StepMapWall.h"

#include <algorithm> /*< for std::sort */
#include <cmath>     /*< for std::sqrt, std::pow */
#include <iomanip>   /*< for std::setw() */
#include <queue>

namespace MazeLib {

const WallIndex StepMapWall::START_WALL_INDEX = WallIndex(0, 0, 1);

void StepMapWall::print(const Maze& maze,
                        const WallIndexes& indexes,
                        const bool show_full_step,
                        std::ostream& os) const {
  const int maze_size = MAZE_SIZE;
  const auto find = [&](const WallIndex& i) {
    return std::find(indexes.cbegin(), indexes.cend(), i) != indexes.cend();
  };
  step_t max_step = 0;
  for (const auto step : step_map)
    if (step != STEP_MAX)
      max_step = std::max(max_step, step);
  const bool simple = (max_step < 999);
  const step_t scaler =
      step_table_diag[MAZE_SIZE * 2 - 1] - step_table_diag[MAZE_SIZE * 2 - 2];
  /* start to draw maze */
  for (int8_t y = maze_size - 1; y >= -1; --y) {
    /* Horizontal Wall Line */
    for (int8_t x = 0; x < maze_size; ++x) {
      /* Pillar */
      os << '+';
      /* Horizontal Wall */
      const auto w = maze.isWall(x, y, Direction::North);
      const auto k = maze.isKnown(x, y, Direction::North);
      const auto i = WallIndex(Position(x, y), Direction::North);
      auto s = getStep(i);
      s = std::min(simple ? s : s / scaler, show_full_step ? 99999 : 999);
      const auto f = find(i);
      if (w)
        os << (show_full_step ? "-----" : "---");
      else
        os << (f ? "\e[43m" C_BL : (s == 0 ? C_YE : (k ? C_BL : C_RE)))
           << std::setw(show_full_step ? 5 : 3) << s << C_NO;
    }
    os << '+' << std::endl;
    /* Vertical Wall Line */
    if (y != -1) {
      os << (show_full_step ? "|  " : "| ");
      for (int8_t x = 0; x < maze_size; ++x) {
        /* Cell */
        os << ' ';
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::East);
        const auto k = maze.isKnown(x, y, Direction::East);
        const auto i = WallIndex(Position(x, y), Direction::East);
        auto s = getStep(i);
        s = std::min(simple ? s : s / scaler, show_full_step ? 99999 : 999);
        const auto f = find(i);
        if (w)
          os << (show_full_step ? "  |  " : " | ");
        else
          os << (f ? "\e[43m" C_BL : (s == 0 ? C_YE : (k ? C_BL : C_RE)))
             << std::setw(show_full_step ? 5 : 3) << s << C_NO;
      }
      os << std::endl;
    }
  }
}
void StepMapWall::print(const Maze& maze,
                        const Directions& shortest_dirs,
                        const WallIndex& start,
                        const bool show_full_step,
                        std::ostream& os) const {
  auto i = start;
  WallIndexes shortest_indexes;
  shortest_indexes.reserve(shortest_dirs.size() + 1);
  shortest_indexes.push_back(i);
  for (const auto d : shortest_dirs) {
    i = i.next(d);
    shortest_indexes.push_back(i);
  }
  print(maze, shortest_indexes, show_full_step, os);
}
void StepMapWall::printPath(const Maze& maze,
                            const WallIndexes& indexes,
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
                            const Directions& shortest_dirs,
                            const WallIndex& start,
                            std::ostream& os) const {
  auto i = start;
  WallIndexes shortest_indexes;
  shortest_indexes.reserve(shortest_dirs.size() + 1);
  shortest_indexes.push_back(i);
  for (const auto d : shortest_dirs) {
    i = i.next(d);
    shortest_indexes.push_back(i);
  }
  printPath(maze, shortest_indexes, os);
}
void StepMapWall::update(const Maze& maze,
                         const WallIndexes& dest,
                         const bool known_only,
                         const bool simple) {
  /* 計算を高速化するため，迷路の大きさを制限 */
  int8_t min_x = maze.getMinX();
  int8_t max_x = maze.getMaxX();
  int8_t min_y = maze.getMinY();
  int8_t max_y = maze.getMaxY();
  for (const auto p : dest) { /*< ゴールを含めないと導出不可能になる */
    min_x = std::min(p.x, min_x);
    max_x = std::max(p.x, max_x);
    min_y = std::min(p.y, min_y);
    max_y = std::max(p.y, max_y);
  }
  min_x -= 1, min_y -= 1, max_x += 2, max_y += 2; /*< 外周を許す */
  /* 全区画のステップを最大値に設定 */
  const auto step = STEP_MAX;
  step_map.fill(step);
  /* ステップの更新予約のキュー */
  std::queue<WallIndex> q;
  /* destのステップを0とする */
  for (const auto i : dest)
    if (i.isInsideOfField())
      step_map[i.getIndex()] = 0, q.push(i);
  /* ステップの更新がなくなるまで更新処理 */
  while (!q.empty()) {
#if MAZE_DEBUG_PROFILING
    queue_size_max = std::max(queue_size_max, q.size());
#endif
    /* 注目する壁を取得 */
    const auto focus = q.front();
    q.pop();
    /* 計算を高速化するため展開範囲を制限 */
    if (focus.x > max_x || focus.y > max_y || focus.x < min_x ||
        focus.y < min_y)
      continue;
    const auto focus_step = step_map[focus.getIndex()];
    /* 周辺を走査 */
    for (const auto d : focus.getNextDirection6()) {
      const auto& step_table =
          (d.isAlong() ? step_table_along : step_table_diag);
      /* 直線で行けるところまで更新する */
      auto next = focus;
      for (int8_t i = 1;; ++i) {
        next = next.next(d); /*< 移動 */
        /* 壁あり or 既知壁のみで未知壁 ならば次へ */
        if (maze.isWall(next) || (known_only && !maze.isKnown(next)))
          break;
        /* 直線加速を考慮したステップを算出 */
        const auto next_step = focus_step + (simple ? i : step_table[i]);
        const auto next_index = next.getIndex();
        if (step_map[next_index] <= next_step)
          break;                          /*< 更新の必要がない */
        step_map[next_index] = next_step; /*< 更新 */
        q.push(next); /*< 再帰的に更新され得るのでキューにプッシュ */
      }
    }
  }
}
Directions StepMapWall::calcShortestDirections(const Maze& maze,
                                               const WallIndex& start,
                                               const WallIndexes& dest,
                                               const bool known_only,
                                               const bool simple) {
  /* ステップマップを更新 */
  update(maze, dest, known_only, simple);
  WallIndex end;
  const auto shortest_dirs =
      getStepDownDirections(maze, start, end, known_only, false);
  /* ゴール判定 */
  return step_map[end.getIndex()] == 0 ? shortest_dirs : Directions{};
}
Directions StepMapWall::getStepDownDirections(const Maze& maze,
                                              const WallIndex& start,
                                              WallIndex& end,
                                              const bool known_only,
                                              const bool break_unknown
                                              __attribute__((unused))) const {
  /* 最短経路となるスタートからの方向列 */
  Directions shortest_dirs;
  /* start から順にステップマップを下る */
  end = start;
  /* 確認 */
  if (!start.isInsideOfField())
    return {};
  while (1) {
    /* 周辺の走査; 未知壁の有無と，最小ステップの方向を求める */
    auto min_d = Direction::Max;
    auto min_step = STEP_MAX;
    /* 周辺を走査 */
    for (const auto d : end.getNextDirection6()) {
      auto next = end;     /*< 隣接 */
      next = next.next(d); /*< 移動 */
      /* 壁あり or 既知壁のみで未知壁 ならば次へ */
      if (maze.isWall(next) || (known_only && !maze.isKnown(next)))
        continue;
      /* min_step よりステップが小さければ更新 (同じなら更新しない) */
      const auto next_step = step_map[next.getIndex()];
      if (min_step <= next_step)
        continue;
      min_step = next_step;
      min_d = d;
    }
    /* 現在地のステップより大きかったらなんかおかしい */
    if (step_map[end.getIndex()] <= min_step)
      break;
    end = end.next(min_d);           //< 位置を更新
    shortest_dirs.push_back(min_d);  //< 既知区間移動
  }
  return shortest_dirs;
}
WallIndexes StepMapWall::convertDestinations(const Maze& maze,
                                             const Positions& positions) {
  WallIndexes dest;
  for (const auto p : positions)
    for (const auto d : Direction::Along4)
      if (!maze.isWall(p, d))
        dest.push_back(WallIndex(p, d));
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
      maze_loge << "invalid direction" << std::endl;
      return Direction::Max;
  }
}
Directions StepMapWall::convertWallIndexDirectionsToPositionDirections(
    const Directions& src) {
  if (src.size() < 2)
    return {};
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
                                           Directions& shortest_dirs,
                                           const WallIndex& start) {
  auto i = start;
  for (const auto d : shortest_dirs)
    i = i.next(d);
  if (shortest_dirs.size()) {
    const auto d = shortest_dirs.back();
    while (1) {
      i = i.next(d);
      if (maze.isWall(i))
        break;
      shortest_dirs.push_back(d);
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
static StepMapWall::step_t gen_cost_impl(const int i,
                                         const float am,
                                         const float vs,
                                         const float vm,
                                         const float seg) {
  const auto d = seg * i; /*< i 区画分の走行距離 */
  /* グラフの面積から時間を求める */
  const auto d_thr = (vm * vm - vs * vs) / am; /*< 最大速度に達する距離 */
  if (d < d_thr)
    return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 1000; /*< 三角加速 */
  else
    return (am * d + (vm - vs) * (vm - vs)) / (am * vm) * 1000; /*< 台形加速 */
}
void StepMapWall::calcStraightStepTable() {
  const float vs = 420.0f;    /*< 基本速度 [mm/s] */
  const float am_a = 4200.0f; /*< 最大加速度 [mm/s/s] */
  const float am_d = 3600.0f; /*< 最大加速度(斜め) [mm/s/s] */
  const float vm_a = 1500.0f; /*< 飽和速度 [mm/s] */
  const float vm_d = 1200.0f; /*< 飽和速度(斜め) [mm/s] */
  const float seg_a = 90.0f;  /*< 1区画の長さ [mm] */
  const float seg_d = 45.0f * std::sqrt(2.0f); /*< 1区画の長さ(斜め) [mm] */
  const float t_slalom = 388.0f; /*< FV90ターンの時間 [ms] */
  step_table_along[0] = step_table_diag[0] = 0; /*< [0] は使用しない */
  for (int i = 1; i < MAZE_SIZE * 2; ++i) {
    /* 斜めと斜めなしが交互に来るので、どちらかでターンのコストを考慮 */
    step_table_along[i] = gen_cost_impl(i, am_a, vs, vm_a, seg_a);
    step_table_diag[i] = t_slalom + gen_cost_impl(i - 1, am_d, vs, vm_d, seg_d);
  }
  /* コストの合計が 65,535 [ms] を超えないようにスケーリング */
  const float scaling_factor = 2;
  for (int i = 0; i < MAZE_SIZE * 2; ++i) {
    step_table_along[i] /= scaling_factor;
    step_table_diag[i] /= scaling_factor;
  }
}

};  // namespace MazeLib
