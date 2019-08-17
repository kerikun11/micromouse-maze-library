/**
 * @file StepMap.cpp
 * @brief マイクロマウスの迷路のステップマップを扱うクラス
 * @author KERI (Github: kerikun11)
 * @url https://kerikeri.top/
 * @date 2017.11.05
 */
#include "StepMap.h"

#include <algorithm> /*< for std::sort */
#include <cmath>     /*< for std::sqrt, std::pow */
#include <iomanip>   /*< for std::setw() */
#include <queue>

namespace MazeLib {

#define STEP_MAP_USE_PRIORITY_QUEUE 1

StepMap::StepMap() {
  calcStraightStepTable();
  reset();
}
void StepMap::reset(const step_t step) {
  for (int8_t y = 0; y < MAZE_SIZE; ++y)
    for (int8_t x = 0; x < MAZE_SIZE; ++x)
      setStep(x, y, step); //< ステップをクリア
}
step_t StepMap::getStep(const int8_t x, const int8_t y) const {
  /* (x, y) がフィールド内か確認 */
  if (x < 0 || y < 0 || x > MAZE_SIZE - 1 || y > MAZE_SIZE - 1) {
    logw << "referred to out of field: " << Vector(x, y) << std::endl;
    return STEP_MAX;
  }
  return step_map[y][x];
}
bool StepMap::setStep(const int8_t x, const int8_t y, const step_t step) {
  /* (x, y) がフィールド内か確認 */
  if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) {
    logw << "referred to out of field: " << Vector(x, y) << std::endl;
    return false;
  }
  step_map[y][x] = step;
  return true;
}
void StepMap::print(std::ostream &os, const Maze &maze, const Vector v,
                    const Dir d) const {
  os << std::endl;
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        os << C_CY << std::setw(3) << std::min(getStep(x, y), (step_t)999)
           << C_NO;
        if ((v == Vector(x, y) && d == Dir::West) ||
            (v == Vector(x, y).next(Dir::East) && d == Dir::East))
          os << C_YE << d.toChar() << C_NO;
        else
          os << (maze.isKnown(x, y, Dir::East)
                     ? (maze.isWall(x, y, Dir::East) ? "|" : " ")
                     : (C_RE "." C_NO));
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      os << "+";
      if ((v == Vector(x, y) && d == Dir::North) ||
          (v == Vector(x, y).next(Dir::South) && d == Dir::South))
        os << " " << C_YE << d.toChar() << C_NO << " ";
      else
        os << (maze.isKnown(x, y, Dir::South)
                   ? (maze.isWall(x, y, Dir::South) ? "---" : "   ")
                   : (C_RE " . " C_NO));
    }
    os << "+" << std::endl;
  }
}
void StepMap::update(const Maze &maze, const Vectors &dest,
                     const bool known_only, const bool simple) {
  /* min max */
  int8_t min_x = maze.getMinX();
  int8_t max_x = maze.getMaxX();
  int8_t min_y = maze.getMinY();
  int8_t max_y = maze.getMaxY();
  for (const auto v : dest) {
    min_x = std::min(v.x, min_x);
    max_x = std::max(v.x, max_x);
    min_y = std::min(v.y, min_y);
    max_y = std::max(v.y, max_y);
  }
  /* 全区画のステップを最大値に設定 */
  reset();
  /* ステップの更新予約のキュー */
  std::queue<Vector> q;
  /* destのステップを0とする */
  for (const auto v : dest) {
    setStep(v, 0);
    q.push(v);
  }
  /* ステップの更新がなくなるまで更新処理 */
  while (!q.empty()) {
    /* 注目する区画を取得 */
    const Vector focus = q.front();
    q.pop();
    const auto focus_step = getStep(focus);
    /* 周辺を走査 */
    for (const auto d : Dir::ENWS()) {
      auto next = focus;
      /* 直線で行けるところまで更新する */
      for (int8_t i = 1; i < MAZE_SIZE * 2; ++i) {
        if (known_only && !maze.isKnown(next, d))
          break; /*< known_only で未知壁なら更新はしない */
        if (maze.isWall(next, d))
          break; /*< 壁があったら更新はしない */
        if (next.x > max_x + 2 || next.y > max_y + 2 || next.x + 1 < min_x ||
            next.y + 1 < min_y)
          break; /*< 注目範囲外なら更新しない */
        /* 移動 */
        next = next.next(d);
        /* 直線加速を考慮したステップを算出 */
        const auto next_step = focus_step + (simple ? 1 : step_table_along[i]);
        if (getStep(next) <= next_step)
          // continue;               /*< 更新の必要がない */
          break;                  /*< 更新の必要がない */
        setStep(next, next_step); /*< 更新 */
        q.push(next); /*< 再帰的に更新され得るのでキューにプッシュ */
        /* 軽量版なら break */
        if (simple)
          break;
      }
    }
  }
}
bool StepMap::calcShortestDirs(const Maze &maze, Dirs &shortestDirs,
                               const bool known_only, const bool simple) {
  /* 目的地を作成 */
  update(maze, maze.getGoals(), known_only, simple);
  VecDir end;
  /* 最短経路を導出 */
  shortestDirs = calcNextDirsKnown(maze, {maze.getStart(), Dir::North}, end,
                                   false, known_only);
  if (getStep(end.first) == 0)
    return true;
  return false;
}
const Vector StepMap::calcNextDirsAdv(Maze &maze, const Vectors &dest,
                                      const Vector vec, const Dir dir,
                                      Dirs &nextDirsKnown,
                                      Dirs &nextDirCandidates) {
  /* ステップマップの更新 */
  update(maze, dest, false, false);
  /* 事前に進む候補を決定する */
  const auto v = calcNextDirs(maze, vec, dir, nextDirsKnown, nextDirCandidates);
  Dirs ndcs; //< Next Dir Candidates
  WallLogs cache;
  while (1) {
    if (nextDirCandidates.empty())
      break;
    const Dir d = nextDirCandidates[0]; //< 行きたい方向
    ndcs.push_back(d);                  //< 候補に入れる
    if (maze.isKnown(v, d))
      break;                               //< 既知なら終わり
    cache.push_back(WallLog(v, d, false)); //< 壁をたてるのでキャッシュしておく
    maze.setWall(v, d, true);  //< 壁をたてる
    maze.setKnown(v, d, true); //< 既知とする
    Dirs tmp_nds;
    // 行く方向を計算しなおす
    update(maze, dest, false, false);
    calcNextDirs(maze, v, d, tmp_nds, nextDirCandidates);
    if (!tmp_nds.empty())
      nextDirCandidates = tmp_nds; //< 既知区間になった場合
  }
  // キャッシュを復活
  for (const auto wl : cache) {
    maze.setWall(Vector(wl), wl.d, false);
    maze.setKnown(Vector(wl), wl.d, false);
  }
  nextDirCandidates = ndcs;
  return v;
}
const Vector StepMap::calcNextDirs(const Maze &maze, const Vector start_v,
                                   const Dir start_d, Dirs &nextDirsKnown,
                                   Dirs &nextDirCandidates) const {
  VecDir end;
  nextDirsKnown = calcNextDirsKnown(maze, {start_v, start_d}, end, true, false);
  nextDirCandidates = calcNextDirCandidates(maze, end);
  return end.first;
}
const Dirs StepMap::calcNextDirsKnown(const Maze &maze, const VecDir start,
                                      VecDir &focus, const bool break_unknown,
                                      const bool known_only) const {
  /* ステップマップから既知区間進行方向列を生成 */
  Dirs nextDirsKnown;
  /* start から順にステップマップを下って行く */
  focus = start;
  while (1) {
    /* 周辺の走査; 未知壁の有無と，最小ステップの方向を求める */
    auto min_d = Dir::Max;
    auto min_step = STEP_MAX;
    for (const auto d : {focus.second + Dir::Front, focus.second + Dir::Left,
                         focus.second + Dir::Right, focus.second + Dir::Back}) {
      /* 壁があったらスキップ */
      if (maze.isWall(focus.first, d))
        continue;
      /* known_only で未知壁ならばスキップ */
      if (known_only && !maze.isKnown(focus.first, d))
        continue;
      /* break_unknown で未知壁ならば既知区間は終了 */
      if (break_unknown && !maze.isKnown(focus.first, d))
        return nextDirsKnown;
      /* min_step よりステップが小さければ更新 (同じなら更新しない) */
      const auto next = focus.first.next(d);
      const auto next_step = getStep(next);
      if (min_step > next_step) {
        min_step = next_step;
        min_d = d;
      }
    }
    /* focus_step より大きかったらなんかおかしい */
    if (getStep(focus.first) <= min_step)
      break;                               //< 永遠ループ防止
    nextDirsKnown.push_back(min_d);        //< 既知区間移動
    focus.first = focus.first.next(min_d); //< 位置を更新
    focus.second = min_d;
  }
  return nextDirsKnown;
}
const Dirs StepMap::calcNextDirCandidates(const Maze &maze,
                                          const VecDir focus) const {
  /* 方向の候補を抽出 */
  Dirs dirs;
  for (const auto d : {focus.second + Dir::Front, focus.second + Dir::Left,
                       focus.second + Dir::Right, focus.second + Dir::Back}) {
    const auto next = focus.first.next(d);
    if (!maze.isWall(focus.first, d) && getStep(next) != STEP_MAX)
      dirs.push_back(d);
  }
  /* ステップが小さい順に並べ替え */
  std::sort(dirs.begin(), dirs.end(), [&](const Dir d1, const Dir d2) {
    return getStep(focus.first.next(d1)) <
           getStep(focus.first.next(d2)); /*< 低コスト優先 */
  });
  /* 未知壁優先で並べ替え, これがないと探索時間増大 */
  std::sort(dirs.begin(), dirs.end(),
            [&](const Dir d1 __attribute__((unused)), const Dir d2) {
              return !maze.unknownCount(focus.first.next(d2));
            });
  return dirs;
}
static step_t gen_cost_impl(const int i, const float am, const float vs,
                            const float vm, const float seg) {
  const auto d = seg * i; /*< i 区画分の走行距離 */
  /* グラフの面積から時間を求める */
  const auto d_thr = (vm * vm - vs * vs) / am; /*< 最大速度に達する距離 */
  if (d < d_thr)
    return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 1000; /*< 三角加速 */
  else
    return (am * d + (vm - vs) * (vm - vs)) / (am * vm) * 1000; /*< 台形加速 */
}
void StepMap::calcStraightStepTable() {
  float vs = 450.0f;    /*< 基本速度 [mm/s] */
  float am_a = 4800.0f; /*< 最大加速度 [mm/s/s] */
  float vm_a = 1800.0f; /*< 飽和速度 [mm/s] */
  const float seg_a = 90.0f;
  for (int i = 0; i < MAZE_SIZE * 2; ++i) {
    step_table_along[i] = gen_cost_impl(i, am_a, vs, vm_a, seg_a);
  }
  const step_t turn_cost = 280 - step_table_along[1];
  for (int i = 0; i < MAZE_SIZE * 2; ++i) {
    step_table_along[i] += turn_cost;
  }
}

} // namespace MazeLib
