/**
 * @file StepMapWall.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 壁ベースのステップマップを表現するクラス
 * @date 2019-08-17
 */
#include "StepMapWall.h"

#include <algorithm> /*< for std::sort */
#include <cmath>     /*< for std::sqrt, std::pow */
#include <iomanip>   /*< for std::setw() */
#include <queue>

namespace MazeLib {

void StepMapWall::print(const Maze &maze, std::ostream &os) const {
  int maze_size = MAZE_SIZE;
  for (int8_t y = maze_size - 1; y >= -1; --y) {
    for (int8_t x = 0; x <= maze_size; ++x) {
      /* Pillar */
      os << "+";
      if (x == maze_size)
        break;
      /* Horizontal Wall */
      const auto w = maze.isWall(x, y, Direction::North);
      const auto k = maze.isKnown(x, y, Direction::North);
      const auto i = WallIndex(Position(x, y), Direction::North);
      if (w)
        os << "-----" << C_NO;
      else
        os << (k ? C_CY : C_RE) << std::setw(5)
           << std::min(int(step_map[i.getIndex()]), 99999) << C_NO;
    }
    os << std::endl;
    if (y != -1) {
      os << "|  ";
      for (int8_t x = 0; x < maze_size; ++x) {
        /* Cell */
        os << " ";
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::East);
        const auto k = maze.isKnown(x, y, Direction::East);
        const auto i = WallIndex(Position(x, y), Direction::East);
        if (w)
          os << "  |  " << C_NO;
        else
          os << (k ? C_CY : C_RE) << std::setw(5)
             << std::min(int(step_map[i.getIndex()]), 99999) << C_NO;
      }
      os << std::endl;
    }
  }
}
void StepMapWall::print(const Maze &maze, const WallIndexes &indexes,
                        std::ostream &os) const {
  const auto exists = [&](const WallIndex &i) {
    return std::find(indexes.cbegin(), indexes.cend(), i) != indexes.cend();
  };
  for (int8_t y = MAZE_SIZE - 1; y >= -1; --y) {
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      /* Pillar */
      os << "+";
      /* Horizontal Wall */
      if (exists(WallIndex(x, y, 1)))
        os << C_YE << " X " << C_NO;
      else
        os << (maze.isKnown(x, y, Direction::North)
                   ? (maze.isWall(x, y, Direction::North) ? "---" : "   ")
                   : (C_RE " . " C_NO));
    }
    os << "+" << std::endl;
    if (y != -1) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        /* Cell */
        os << "   ";
        /* Vertical Wall */
        if (exists(WallIndex(x, y, 0)))
          os << C_YE << "X" << C_NO;
        else
          os << (maze.isKnown(x, y, Direction::East)
                     ? (maze.isWall(x, y, Direction::East) ? "|" : " ")
                     : (C_RE "." C_NO));
      }
      os << std::endl;
    }
  }
}
void StepMapWall::print(const Maze &maze, const Directions &shortest_dirs,
                        const WallIndex &start, std::ostream &os) const {
  auto i = start;
  WallIndexes shortest_indexes;
  shortest_indexes.push_back(i);
  for (const auto d : shortest_dirs) {
    i = i.next(d);
    shortest_indexes.push_back(i);
  }
  print(maze, shortest_indexes, os);
}
void StepMapWall::update(const Maze &maze, const WallIndexes &dest,
                         const bool known_only, const bool simple) {
  /* 全区画のステップを最大値に設定 */
  const auto step = STEP_MAX;
  step_map.fill(step);
  /* ステップの更新予約のキュー */
  std::queue<WallIndex> q;
  /* destのステップを0とする */
  for (const auto i : dest)
    step_map[i.getIndex()] = 0, q.push(i);
  /* ステップの更新がなくなるまで更新処理 */
  while (!q.empty()) {
    /* 注目する壁を取得 */
    const auto focus = q.front();
    q.pop();
    const step_t focus_step = step_map[focus.getIndex()];
    /* 周辺を走査 */
    for (const auto d : focus.getNextDirection6()) {
      auto next = focus;
      /* 直線で行けるところまで更新する */
      for (int8_t i = 1; i < MAZE_SIZE * 2; ++i) {
        next = next.next(d); /*< 移動 */
        if (maze.isWall(next) || (known_only && !maze.isKnown(next)))
          break; /*< 壁あり or 既知壁のみで未知壁 ならば次へ */
        /* 直線加速を考慮したステップを算出 */
        const auto next_step = focus_step + (d.isAlong() ? step_table_along[i]
                                                         : step_table_diag[i]);
        if (step_map[next.getIndex()] <= next_step)
          break;                               /*< 更新の必要がない */
        step_map[next.getIndex()] = next_step; /*< 更新 */
        q.push(next); /*< 再帰的に更新され得るのでキューにプッシュ */
        if (simple) /*< 軽量版なら break */
          break;
      }
    }
  }
}
const Directions StepMapWall::calcShortestDirections(const Maze &maze,
                                                     const WallIndex &start,
                                                     const WallIndexes &dest,
                                                     const bool known_only,
                                                     const bool simple) {
  /* ステップマップを更新 */
  update(maze, dest, known_only, simple);
  /* 最短経路となるスタートからの方向列 */
  Directions shortest_dirs;
  /* start から順にステップマップを下る */
  auto focus = start;
  while (1) {
    /* 周辺の走査; 未知壁の有無と，最小ステップの方向を求める */
    auto min_d = Direction::Max;
    auto min_step = STEP_MAX;
    auto min_i = focus;
    /* 周辺を走査 */
    for (const auto d : focus.getNextDirection6()) {
      auto next = focus; /*< 隣接 */
      /* 直線で行けるところまで更新する */
      for (int8_t i = 1; i < MAZE_SIZE * 2; ++i) {
        next = next.next(d); /*< 移動 */
        /* 壁があったら次へ */
        if (maze.isWall(next) || (known_only && !maze.isKnown(next)))
          break;
        /* min_step よりステップが小さければ更新 (同じなら更新しない) */
        const auto next_step = step_map[next.getIndex()];
        if (min_step <= next_step)
          break;
        min_step = next_step;
        min_d = d;
        min_i = next;
      }
    }
    /* focus_step より大きかったらなんかおかしい */
    if (step_map[focus.getIndex()] <= min_step)
      break;
    /* 直線の分だけ移動 */
    while (focus != min_i) {
      focus = focus.next(min_d);      //< 位置を更新
      shortest_dirs.push_back(min_d); //< 既知区間移動
    }
  }
  /* ゴール判定 */
  if (step_map[focus.getIndex()] != 0)
    return {}; //< 失敗
  return shortest_dirs;
}
const WallIndexes StepMapWall::convertDestinations(const Maze &maze,
                                                   const Positions &positions) {
  WallIndexes dest;
  for (const auto p : positions)
    for (const auto d : Direction::getAlong4())
      if (!maze.isWall(p, d))
        dest.push_back(WallIndex(p, d));
  return dest;
}
const Direction StepMapWall::convertDirection(const Direction d,
                                              const WallIndex &i) {
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
    logw << "invalid direction" << std::endl;
    return Direction::Max;
  }
}
const Directions StepMapWall::convertWallIndexDirectionsToPositionDirections(
    const Directions &src, const WallIndex &start) {
  Directions dirs;
  dirs.push_back(Direction::North);
  auto i = start;
  for (const auto d : src) {
    dirs.push_back(convertDirection(d, i));
    i = i.next(d);
  }
  return dirs;
}

static StepMapWall::step_t gen_cost_impl(const int i, const float am,
                                         const float vs, const float vm,
                                         const float seg) {
  const auto d = seg * i; /*< i 区画分の走行距離 */
  /* グラフの面積から時間を求める */
  const auto d_thr = (vm * vm - vs * vs) / am; /*< 最大速度に達する距離 */
  if (d < d_thr)
    return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 100; /*< 三角加速 */
  else
    return (am * d + (vm - vs) * (vm - vs)) / (am * vm) * 100; /*< 台形加速 */
}
void StepMapWall::calcStraightStepTable() {
  const float vs = 450.0f;    /*< 基本速度 [mm/s] */
  const float am_a = 4800.0f; /*< 最大加速度 [mm/s/s] */
  const float am_d = 3600.0f; /*< 最大加速度(斜め) [mm/s/s] */
  const float vm_a = 1800.0f; /*< 飽和速度 [mm/s] */
  const float vm_d = 1200.0f; /*< 飽和速度(斜め) [mm/s] */
  const float seg_a = 90.0f;  /*< 1区画の長さ [mm] */
  const float seg_d = 45.0f * std::sqrt(2); /*< 1区画の長さ(斜め) [mm] */
  for (int i = 0; i < MAZE_SIZE * 2; ++i) {
    step_table_along[i] = gen_cost_impl(i, am_a, vs, vm_a, seg_a);
    step_table_diag[i] = gen_cost_impl(i, am_d, vs, vm_d, seg_d);
  }
  const step_t turn_cost = 280 - step_table_along[1];
  for (int i = 0; i < MAZE_SIZE * 2; ++i) {
    step_table_along[i] += turn_cost;
    step_table_diag[i] += turn_cost;
  }
}

}; // namespace MazeLib