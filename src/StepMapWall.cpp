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

StepMapWall::StepMapWall() {
  calcStraightStepTable();
  reset();
}
void StepMapWall::reset(const step_t step) {
  for (int8_t z = 0; z < 2; ++z)
    for (int8_t y = 0; y < MAZE_SIZE; ++y)
      for (int8_t x = 0; x < MAZE_SIZE; ++x)
        setStep(WallIndex(x, y, z), step); //< ステップをクリア
}
void StepMapWall::print(const Maze &maze, std::ostream &os) const {
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        os << C_CY << std::setw(3)
           << std::min(getStep(WallIndex(x, y, 1)), (step_t)999) << C_NO;
        os << " ";
        os << C_CY << std::setw(3)
           << std::min(getStep(WallIndex(x, y, 0)), (step_t)999) << C_NO;
        os << (maze.isKnown(x, y, Dir::East)
                   ? (maze.isWall(x, y, Dir::East) ? "|" : " ")
                   : (C_RE "." C_NO));
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      os << "+";
      os << (maze.isKnown(x, y, Dir::South)
                 ? (maze.isWall(x, y, Dir::South) ? "-------" : "       ")
                 : (C_RE "  . .  " C_NO));
    }
    os << "+" << std::endl;
  }
}
void StepMapWall::print(const Maze &maze, const WallIndexes &indexes,
                        std::ostream &os) const {
  const auto exists = [&](const WallIndex i) {
    return std::find(indexes.cbegin(), indexes.cend(), i) != indexes.cend();
  };
  for (int8_t y = MAZE_SIZE - 1; y >= -1; --y) {
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      os << "+";
      if (exists(WallIndex(x, y, 1)))
        os << C_YE << " X " << C_NO;
      else
        os << (maze.isKnown(x, y, Dir::North)
                   ? (maze.isWall(x, y, Dir::North) ? "---" : "   ")
                   : (C_RE " . " C_NO));
    }
    os << "+" << std::endl;
    if (y != -1) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        os << "   ";
        if (exists(WallIndex(x, y, 0)))
          os << C_YE << "X" << C_NO;
        else
          os << (maze.isKnown(x, y, Dir::East)
                     ? (maze.isWall(x, y, Dir::East) ? "|" : " ")
                     : (C_RE "." C_NO));
      }
      os << std::endl;
    }
  }
}
void StepMapWall::print(const Maze &maze, const Dirs &shortest_dirs,
                        const WallIndex start, std::ostream &os) const {
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
  /* 迷路の大きさを決定 */
  int8_t min_x = maze.getMinX();
  int8_t max_x = maze.getMaxX();
  int8_t min_y = maze.getMinY();
  int8_t max_y = maze.getMaxY();
  for (const auto v : dest) { /*< ゴールを含めないと導出不可能になる */
    min_x = std::min(v.x, min_x);
    max_x = std::max(v.x, max_x);
    min_y = std::min(v.y, min_y);
    max_y = std::max(v.y, max_y);
  }
  /* 全区画のステップを最大値に設定 */
  reset();
  /* ステップの更新予約のキュー */
  std::queue<WallIndex> q;
  /* destのステップを0とする */
  for (const auto i : dest)
    setStep(i, 0), q.push(i);
  /* ステップの更新がなくなるまで更新処理 */
  while (!q.empty()) {
    /* 注目する壁を取得 */
    const auto focus = q.front();
    q.pop();
    const step_t focus_step = getStep(focus);
    /* 周辺を走査 */
    for (const auto d : focus.getNextDir6()) {
      auto next = focus;
      /* 直線で行けるところまで更新する */
      for (int8_t i = 1; i < MAZE_SIZE * 2; ++i) {
        next = next.next(d); /*< 移動 */
        if (maze.isWall(next) || (known_only && !maze.isKnown(next)))
          break; /*< 壁あり or 既知壁のみで未知壁 ならば次へ */
        if (next.x > max_x + 2 || next.y > max_y + 2 || next.x + 1 < min_x ||
            next.y + 1 < min_y)
          break; /*< 注目範囲外なら更新しない */
        /* 直線加速を考慮したステップを算出 */
        const auto next_step = focus_step + (d.isAlong() ? step_table_along[i]
                                                         : step_table_diag[i]);
        if (getStep(next) <= next_step)
          break;                  /*< 更新の必要がない */
        setStep(next, next_step); /*< 更新 */
        q.push(next); /*< 再帰的に更新され得るのでキューにプッシュ */
        if (simple) /*< 軽量版なら break */
          break;
      }
    }
  }
}
const WallIndexes StepMapWall::convertDestinations(const Maze &maze,
                                                   const Vectors vectors) {
  WallIndexes dest;
  for (const auto v : vectors)
    for (const auto d : Dir::ENWS())
      if (!maze.isWall(v, d))
        dest.push_back(WallIndex(v, d));
  return dest;
}
const Dir StepMapWall::convertDir(const Dir d, const WallIndex i) {
  switch (d) {
  case Dir::East:
  case Dir::North:
  case Dir::West:
  case Dir::South:
    return d;
  case Dir::NorthEast:
    return i.z == 0 ? Dir::North : Dir::East;
  case Dir::SouthWest:
    return i.z == 0 ? Dir::South : Dir::West;
  case Dir::NorthWest:
    return i.z == 0 ? Dir::North : Dir::West;
  case Dir::SouthEast:
    return i.z == 0 ? Dir::South : Dir::East;
  }
  return Dir::Max;
}
const Dirs
StepMapWall::convertWallIndexDirsToVectorDirs(const Dirs src,
                                              const WallIndex start) {
  Dirs dirs;
  dirs.push_back(Dir::North);
  auto i = start;
  for (const auto d : src) {
    dirs.push_back(convertDir(d, i));
    i = i.next(d);
  }
  return dirs;
}
bool StepMapWall::calcShortestDirs(const Maze &maze, Dirs &shortest_dirs,
                                   const bool known_only, const bool simple) {
  /* 目的地を作成 */
  WallIndexes dest = convertDestinations(maze, maze.getGoals());
  update(maze, dest, known_only, simple);
  WallIndex end;
  shortest_dirs =
      calcDirsStepDown(maze, WallIndex(0, 0, 1), end, false, known_only);
  if (getStep(end) != 0)
    return false; /*< ゴールに到達していない */
  return true;
}

static step_t gen_cost_impl(const int i, const float am, const float vs,
                            const float vm, const float seg) {
  const auto d = seg * i; /*< i 区画分の走行距離 */
  /* グラフの面積から時間を求める */
  const auto d_thr = (vm * vm - vs * vs) / am; /*< 最大速度に達する距離 */
  if (d < d_thr)
    return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 100; /*< 三角加速 */
  else
    return (am * d + (vm - vs) * (vm - vs)) / (am * vm) * 100; /*< 台形加速 */
}
void StepMapWall::calcStraightStepTable() {
  float vs = 450.0f;    /*< 基本速度 [mm/s] */
  float am_a = 4800.0f; /*< 最大加速度 [mm/s/s] */
  float am_d = 3600.0f; /*< 最大加速度(斜め) [mm/s/s] */
  float vm_a = 1800.0f; /*< 飽和速度 [mm/s] */
  float vm_d = 1200.0f; /*< 飽和速度(斜め) [mm/s] */
  const float seg_a = 90.0f;
  const float seg_d = 45.0f * std::sqrt(2);
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
const Dirs StepMapWall::calcDirsStepDown(const Maze &maze,
                                         const WallIndex start,
                                         WallIndex &focus,
                                         const bool break_unknown,
                                         const bool known_only) const {
  /* ステップマップから既知区間進行方向列を生成 */
  Dirs nextDirsKnown;
  /* start から順にステップマップを下って行く */
  focus = start;
  while (1) {
    /* 周辺の走査; 未知壁の有無と，最小ステップの方向を求める */
    auto min_d = Dir::Max;
    auto min_step = STEP_MAX;
    auto min_i = focus;
    /* 周辺を走査 */
    for (const auto d : focus.getNextDir6()) {
      auto next = focus; /*< 隣接 */
      /* break_unknown で未知壁ならば既知区間は終了 */
      if (break_unknown && !maze.isKnown(next.next(d)))
        return nextDirsKnown;
      /* 直線で行けるところまで更新する */
      for (int8_t i = 1; i < MAZE_SIZE * 2; ++i) {
        // for (int8_t i = 1; i < 2; ++i) {
        next = next.next(d); /*< 移動 */
        /* 壁があったら次へ */
        if (maze.isWall(next) || (known_only && !maze.isKnown(next)))
          break;
        /* min_step よりステップが小さければ更新 (同じなら更新しない) */
        const auto next_step = getStep(next);
        if (min_step <= next_step)
          break;
        min_step = next_step;
        min_d = d;
        min_i = next;
      }
    }
    /* focus_step より大きかったらなんかおかしい */
    if (getStep(focus) <= min_step)
      break;
    /* 直線の分だけ移動 */
    while (focus != min_i) {
      focus = focus.next(min_d);      //< 位置を更新
      nextDirsKnown.push_back(min_d); //< 既知区間移動
    }
  }
  return nextDirsKnown;
}
const Dirs StepMapWall::calcNextDirCandidates(const Maze &maze,
                                              const WallIndex focus,
                                              bool prior_unknown) const {
  /* 方向の候補を抽出 */
  Dirs dirs;
  for (const auto d : focus.getNextDir6()) {
    const auto next = focus.next(d);
    if (!maze.isWall(next) && getStep(next) != STEP_MAX)
      dirs.push_back(d);
  }
  /* ステップが小さい順に並べ替え */
  std::sort(dirs.begin(), dirs.end(), [&](const Dir d1, const Dir d2) {
    return getStep(focus.next(d1)) <
           getStep(focus.next(d2)); /*< 低コスト優先 */
  });
  /* 未知壁優先で並べ替え, これがないと探索時間増大 */
  if (prior_unknown)
    std::sort(dirs.begin(), dirs.end(),
              [&](const Dir d1 __attribute__((unused)), const Dir d2) {
                return maze.isKnown(focus.next(d2));
              });
  return dirs;
}

}; // namespace MazeLib
