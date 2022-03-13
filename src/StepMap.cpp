/**
 * @file StepMap.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief マイクロマウスの迷路のステップマップを扱うクラス
 * @copyright Copyright (c) 2017 Ryotaro Onuki
 * @date 2017.11.05
 */
#include "MazeLib/StepMap.h"

#include <algorithm> /*< for std::sort */
#include <cmath>     /*< for std::sqrt, std::pow */
#include <iomanip>   /*< for std::setw() */
#include <queue>

namespace MazeLib {

StepMap::StepMap() {
  calcStraightStepTable();
  reset();
}
void StepMap::print(const Maze& maze,
                    const Position& p,
                    const Direction d,
                    std::ostream& os) const {
  return print(maze, {d}, p.next(d + Direction::Back), os);
}
void StepMap::print(const Maze& maze,
                    const Directions& dirs,
                    const Position& start,
                    std::ostream& os) const {
  /* preparation */
  std::vector<Pose> path;
  Position p = start;
  for (const auto d : dirs)
    path.push_back({p, d}), p = p.next(d);
  const int maze_size = MAZE_SIZE;
  step_t max_step = 0;
  for (const auto step : step_map)
    if (step != STEP_MAX)
      max_step = std::max(max_step, step);
  const bool simple = (max_step < 999);
  const auto find = [&](const WallIndex& i) {
    return std::find_if(path.cbegin(), path.cend(), [&](const Pose& pose) {
      return WallIndex(pose.p, pose.d) == i;
    });
  };
  /* start to draw maze */
  for (int8_t y = maze_size; y >= 0; --y) {
    if (y != maze_size) {
      for (uint8_t x = 0; x <= maze_size; ++x) {
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::West);
        const auto k = maze.isKnown(x, y, Direction::West);
        const auto it = find(WallIndex(Position(x, y), Direction::West));
        if (it != path.cend())
          os << "\e[43m\e[34m" << it->d << C_NO;
        else
          os << (k ? (w ? "|" : " ") : (C_RE "." C_NO));
        /* Cell */
        if (x != maze_size) {
          if (getStep(x, y) == STEP_MAX)
            os << C_CY << "999" << C_NO;
          else if (getStep(x, y) == 0)
            os << C_YE << std::setw(3) << getStep(x, y) << C_NO;
          else if (simple)
            os << C_CY << std::setw(3) << getStep(x, y) << C_NO;
          else
            os << C_CY << std::setw(3) << getStep(x, y) / 100 << C_NO;
        }
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < maze_size; ++x) {
      /* Pillar */
      os << '+';
      /* Horizontal Wall */
      const auto w = maze.isWall(x, y, Direction::South);
      const auto k = maze.isKnown(x, y, Direction::South);
      const auto it = find(WallIndex(Position(x, y), Direction::South));
      if (it != path.cend())
        os << "\e[43m\e[34m " << it->d << ' ' << C_NO;
      else
        os << (k ? (w ? "---" : "   ") : (C_RE " . " C_NO));
    }
    /* Last Pillar */
    os << '+' << std::endl;
  }
}
void StepMap::printFull(const Maze& maze,
                        const Position& p,
                        const Direction d,
                        std::ostream& os) const {
  return printFull(maze, {d}, p.next(d + Direction::Back), os);
}
void StepMap::printFull(const Maze& maze,
                        const Directions& dirs,
                        const Position& start,
                        std::ostream& os) const {
  std::vector<Pose> path;
  Position p = start;
  for (const auto d : dirs) {
    p = p.next(d);
    path.push_back({p, d});
  }
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        os << C_CY << std::setw(5) << std::min((int)getStep(x, y), 99999)
           << C_NO;
        bool found = false;
        for (const auto pose : path) {
          const auto p = pose.p;
          const auto d = pose.d;
          if ((p == Position(x, y) && d == Direction::West) ||
              (p == Position(x, y).next(Direction::East) &&
               d == Direction::East)) {
            os << C_YE << d.toChar() << C_NO;
            found = true;
            break;
          }
        }
        if (!found)
          os << (maze.isKnown(x, y, Direction::East)
                     ? (maze.isWall(x, y, Direction::East) ? "|" : " ")
                     : (C_RE "." C_NO));
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      os << '+';
      bool found = false;
      for (const auto pose : path) {
        const auto p = pose.p;
        const auto d = pose.d;
        if ((p == Position(x, y) && d == Direction::North) ||
            (p == Position(x, y).next(Direction::South) &&
             d == Direction::South)) {
          os << "  " << C_YE << d.toChar() << C_NO << "  ";
          found = true;
          break;
        }
      }
      if (!found)
        os << (maze.isKnown(x, y, Direction::South)
                   ? (maze.isWall(x, y, Direction::South) ? "-----" : "     ")
                   : (C_RE " . . " C_NO));
    }
    os << '+' << std::endl;
  }
}
void StepMap::update(const Maze& maze,
                     const Positions& dest,
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
  /* 直線優先 */
  const int max_straight = simple ? 1 : MAZE_SIZE * 2;
  /* 全区画のステップを最大値に設定 */
  reset();
  /* ステップの更新予約のキュー */
  std::queue<Position> q;
  /* destのステップを0とする */
  for (const auto p : dest)
    if (p.isInsideOfField())
      setStep(p, 0), q.push(p);
  /* ステップの更新がなくなるまで更新処理 */
  while (!q.empty()) {
    /* 注目する区画を取得 */
    const auto focus = q.front();
    q.pop();
    /* 計算を高速化するため展開範囲を制限 */
    if (focus.x > max_x || focus.y > max_y || focus.x < min_x ||
        focus.y < min_y)
      continue;
    const auto focus_step = step_map[focus.getIndex()];
    /* 周辺を走査 */
    for (const auto d : Direction::Along4) {
      /* 直線で行けるところまで更新する */
      auto next = focus;
      for (int8_t i = 1;; ++i) {
        /* 壁あり or 既知壁のみで未知壁 ならば次へ */
        const auto next_wi = WallIndex(next, d);
        if (maze.isWall(next_wi) || (known_only && !maze.isKnown(next_wi)))
          break;
        next = next.next(d); /*< 移動 */
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
Directions StepMap::calcShortestDirections(const Maze& maze,
                                           const Position& start,
                                           const Positions& dest,
                                           const bool known_only,
                                           const bool simple) {
  /* ステップマップを更新 */
  update(maze, dest, known_only, simple);
  Pose end;
  const auto shortest_dirs = getStepDownDirections(
      maze, {start, Direction::Max}, end, known_only, false);
  /* ゴール判定 */
  return step_map[end.p.getIndex()] == 0 ? shortest_dirs : Directions{};
}
Pose StepMap::calcNextDirections(const Maze& maze,
                                 const Pose& start,
                                 Directions& nextDirectionsKnown,
                                 Directions& nextDirectionCandidates) const {
  Pose end;
  nextDirectionsKnown = getStepDownDirections(maze, start, end, false, true);
  nextDirectionCandidates = getNextDirectionCandidates(maze, end);
  return end;
}
Directions StepMap::getStepDownDirections(const Maze& maze,
                                          const Pose& start,
                                          Pose& end,
                                          const bool known_only,
                                          const bool break_unknown) const {
  /* ステップマップから既知区間進行方向列を生成 */
  Directions shortest_dirs;
  /* start から順にステップマップを下る */
  end = start;
  /* 確認 */
  if (!start.p.isInsideOfField())
    return {};
  while (1) {
    /* 周辺の走査; 未知壁の有無と，最小ステップの方向を求める */
    auto min_pose = end;
    auto min_step = STEP_MAX;
    for (const auto d : Direction::Along4) {
      auto next = end.p; /*< 隣接 */
      for (int8_t i = 1; i < MAZE_SIZE; ++i) {
        /* 壁あり or 既知壁のみで未知壁 ならば次へ */
        if (maze.isWall(next, d) || (known_only && !maze.isKnown(next, d)))
          break;
        next = next.next(d); /*< 隣接区画へ移動 */
        /* 現時点の min_step よりステップが小さければ更新 */
        const auto next_step = step_map[next.getIndex()];
        if (min_step <= next_step)
          break;
        min_step = next_step;
        min_pose = Pose{next, d};
      }
    }
    /* 現在地よりステップが大きかったらなんかおかしい */
    if (step_map[end.p.getIndex()] <= min_step)
      break;
    /* 移動分を結果に追加 */
    while (end.p != min_pose.p) {
      /* break_unknown のとき，未知壁を含むならば既知区間は終了 */
      if (break_unknown && maze.unknownCount(end.p))
        return shortest_dirs;
      end = end.next(min_pose.d);
      shortest_dirs.push_back(min_pose.d);
    }
  }
  return shortest_dirs;
}
Directions StepMap::getNextDirectionCandidates(const Maze& maze,
                                               const Pose& focus) const {
  /* 直線優先で進行方向の候補を抽出．全方位 STEP_MAX だと空になる */
  Directions dirs;
  for (const auto d : {focus.d + Direction::Front, focus.d + Direction::Left,
                       focus.d + Direction::Right, focus.d + Direction::Back})
    if (!maze.isWall(focus.p, d) && getStep(focus.p.next(d)) != STEP_MAX)
      dirs.push_back(d);
  /* コストの低い順に並べ替え */
  std::sort(dirs.begin(), dirs.end(),
            [&](const Direction d1, const Direction d2) {
              return getStep(focus.p.next(d1)) < getStep(focus.p.next(d2));
            });
#if 1
  /* 未知壁優先で並べ替え(未知壁同士ならばコストが低い順) */
  std::sort(dirs.begin(), dirs.end(),
            [&](const Direction d1, const Direction d2) {
              return (maze.unknownCount(focus.p.next(d1)) &&
                      !maze.unknownCount(focus.p.next(d2)));
            });
#endif
#if 1
  /* 直進優先に並べ替え */
  std::sort(dirs.begin(), dirs.end(),
            [&](const Direction d1, const Direction d2
                __attribute__((unused))) { return d1 == focus.d; });
#endif
  return dirs;
}
void StepMap::appendStraightDirections(const Maze& maze,
                                       Directions& shortest_dirs,
                                       const bool known_only,
                                       const bool diag_enabled) {
  /* ゴール区画までたどる */
  auto p = maze.getStart();
  for (const auto d : shortest_dirs)
    p = p.next(d);
  if (shortest_dirs.size() < 2)
    return;
  auto prev_dir = shortest_dirs[shortest_dirs.size() - 1 - 1];
  auto dir = shortest_dirs[shortest_dirs.size() - 1];
  /* ゴール区画内を行けるところまで直進(斜め考慮)する */
  bool loop = true;
  while (loop) {
    loop = false;
    /* 斜めを考慮した進行方向を列挙する */
    Directions dirs;
    const auto rel_dir = Direction(dir - prev_dir);
    if (diag_enabled && rel_dir == Direction::Left)
      dirs = {Direction(dir + Direction::Right), dir};
    else if (diag_enabled && rel_dir == Direction::Right)
      dirs = {Direction(dir + Direction::Left), dir};
    else
      dirs = {dir};
    /* 候補のうち行ける方向に行く */
    for (const auto d : dirs) {
      if (!maze.isWall(p, d) && (!known_only || maze.isKnown(p, d))) {
        shortest_dirs.push_back(d);
        p = p.next(d);
        prev_dir = dir;
        dir = d;
        loop = true;
        break;
      }
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
static StepMap::step_t gen_cost_impl(const int i,
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
void StepMap::calcStraightStepTable() {
  const float vs = 420.0f;       /*< 基本速度 [mm/s] */
  const float am_a = 4200.0f;    /*< 最大加速度 [mm/s/s] */
  const float vm_a = 1500.0f;    /*< 飽和速度 [mm/s] */
  const float seg_a = 90.0f;     /*< 区画の長さ [mm] */
  const float t_slalom = 287.0f; /*< 小回り90度ターンの時間 [ms] */
  step_table[0] = 0;             /*< [0] は使用しない */
  for (int i = 1; i < MAZE_SIZE; ++i)
    step_table[i] =
        (step_t)t_slalom + gen_cost_impl(i - 1, am_a, vs, vm_a, seg_a);
  /* 最大値を超えないようにスケーリング */
  const float scaling_factor = 2;
  for (int i = 0; i < MAZE_SIZE; ++i)
    step_table[i] /= scaling_factor;
}

}  // namespace MazeLib
