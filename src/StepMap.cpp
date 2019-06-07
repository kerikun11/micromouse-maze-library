/**
 *  @file StepMap.cpp
 *  @brief マイクロマウスの迷路のステップマップを扱うクラス
 *  @author KERI (Github: kerikun11)
 *  @url https://kerikeri.top/
 *  @date 2017.11.05
 */
#include "StepMap.h"

#include <algorithm>
#include <cmath> /*< for std::sqrt, std::pow */
#include <functional>
#include <iomanip> //< for std::setw()
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
    return MAZE_STEP_MAX;
  }
  return stepMap[y][x];
}
bool StepMap::setStep(const int8_t x, const int8_t y, const step_t step) {
  /* (x, y) がフィールド内か確認 */
  if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) {
    logw << "referred to out of field: " << Vector(x, y) << std::endl;
    return false;
  }
  stepMap[y][x] = step;
  return true;
}
void StepMap::print(std::ostream &os, const Maze &maze, const Vector v,
                    const Dir d) const {
  os << std::endl;
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        if (v == Vector(x, y))
          os << " " << C_YELLOW << d.toChar() << C_RESET << " ";
        else
          os << C_CYAN << std::setw(3) << std::min(getStep(x, y), (step_t)999)
             << C_RESET;
        os << (maze.isKnown(x, y, Dir::East)
                   ? (maze.isWall(x, y, Dir::East) ? "|" : " ")
                   : (C_RED "." C_RESET));
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < MAZE_SIZE; ++x)
      os << "+"
         << (maze.isKnown(x, y, Dir::South)
                 ? (maze.isWall(x, y, Dir::South) ? "---" : "   ")
                 : (C_RED " . " C_RESET));
    os << "+" << std::endl;
  }
}
void StepMap::update(const Maze &maze, const Vectors &dest,
                     const bool known_only, const bool diag_enabled) {
  /* min max */
  int8_t max_x = maze.getMaxX();
  int8_t max_y = maze.getMaxY();
  for (const auto v : dest) {
    max_x = std::max(v.x, max_x);
    max_y = std::max(v.y, max_y);
  }
  // 全区画のステップを最大値に設定
  reset();
  // となりの区画のステップが更新されたので更新が必要かもしれない区画のキュー
#if STEP_MAP_USE_PRIORITY_QUEUE
  std::function<bool(const Vector v1, const Vector v2)> greater =
      [&](const Vector v1, const Vector v2) {
        return getStep(v1) > getStep(v2);
      };
  std::priority_queue<Vector, std::vector<Vector>, decltype(greater)> q(
      greater);
#else
  std::queue<Vector> q;
#endif
  // destに含まれる区画のステップを0とする
  for (const auto v : dest) {
    setStep(v, 0);
    q.push(v);
  }
#define CONFIG_FULL_UPDATE 0
  // ステップの更新がなくなるまで更新処理
  while (!q.empty()) {
    // 注目する区画を取得
#if STEP_MAP_USE_PRIORITY_QUEUE
    const Vector focus = q.top();
#else
    const Vector focus = q.front();
#endif
    q.pop();
    const step_t focus_step = getStep(focus);
    // 4方向更新がないか調べる
    for (const auto d : Dir::ENWS()) {
      if (maze.isWall(focus, d))
        continue; //< 壁があったら更新はしない
      if (known_only && !maze.isKnown(focus, d))
        continue; //< known_only で未知壁なら更新はしない
      if (focus.x > max_x + 1 || focus.y > max_y + 1)
        continue; //< 注目範囲外なら更新しない
      // 直線で行けるところまで更新する
      Vector next = focus;
      for (int8_t i = 0; i < MAZE_SIZE; ++i) {
        if (maze.isWall(next, d))
          break; //< 壁があったら更新はしない
        if (known_only && !maze.isKnown(next, d))
          break; //< known_only で未知壁なら更新はしない
        // となりの区画のステップが注目する区画のステップよりも大きければ更新
        next = next.next(d); //< となりの区画のステップを取得
        const step_t step = focus_step + straightStepTable[i];
        if (getStep(next) <= step)
#if CONFIG_FULL_UPDATE
          continue;
#else
          break; //< これより先，更新されることはない
#endif
        setStep(next, step);
        q.push(next); //< 再帰的に更新され得るのでキューにプッシュ
      }
      if (!diag_enabled)
        continue; //< 斜めなしの場合
      // 斜め直線で行けるところまで更新する
      next = focus.next(d);
      for (int8_t i = 1; i < MAZE_SIZE * 2; ++i) {
        const Dir next_d = d + Dir::Left * (i & 1);
        if (maze.isWall(next, next_d))
          break; //< 壁があったら更新はしない
        if (known_only && !maze.isKnown(next, next_d))
          break; //< known_only で未知壁なら更新はしない
        // となりの区画のステップが注目する区画のステップよりも大きければ更新
        next = next.next(next_d); //< となりの区画のステップを取得
        const step_t step = focus_step + straightStepTable[i] + 1;
        if (getStep(next) <= step)
#if CONFIG_FULL_UPDATE
          continue;
#else
          break; //< これより先，更新されることはない
#endif
        setStep(next, step);
        q.push(next); //< 再帰的に更新され得るのでキューにプッシュ
      }
      // 斜め直線で行けるところまで更新する
      next = focus.next(d);
      for (int8_t i = 1; i < MAZE_SIZE * 2; ++i) {
        const Dir next_d = d + Dir::Right * (i & 1);
        if (maze.isWall(next, next_d))
          break; //< 壁があったら更新はしない
        if (known_only && !maze.isKnown(next, next_d))
          break; //< known_only で未知壁なら更新はしない
        // となりの区画のステップが注目する区画のステップよりも大きければ更新
        next = next.next(next_d); //< となりの区画のステップを取得
        const step_t step = focus_step + straightStepTable[i] + 1;
        if (getStep(next) <= step)
#if CONFIG_FULL_UPDATE
          continue;
#else
          break; //< これより先，更新されることはない
#endif
        setStep(next, step);
        q.push(next); //< 再帰的に更新され得るのでキューにプッシュ
      }
    }
  }
}
void StepMap::updateSimple(const Maze &maze, const Vectors &dest,
                           const bool known_only) {
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
  // となりの区画のステップが更新されたので更新が必要かもしれない区画のキュー
#if STEP_MAP_USE_PRIORITY_QUEUE
  std::function<bool(const Vector v1, const Vector v2)> greater =
      [&](const Vector v1, const Vector v2) {
        return getStep(v1) > getStep(v2);
      };
  std::priority_queue<Vector, std::vector<Vector>, decltype(greater)> q(
      greater);
#else
  std::queue<Vector> q;
#endif
  // destに含まれる区画のステップを0とする
  for (const auto v : dest) {
    setStep(v, 0);
    q.push(v);
  }
  // ステップの更新がなくなるまで更新処理
  while (!q.empty()) {
    // 注目する区画を取得
#if STEP_MAP_USE_PRIORITY_QUEUE
    const Vector focus = q.top();
#else
    const Vector focus = q.front();
#endif
    q.pop();
    const step_t focus_step = getStep(focus);
    // 4方向更新がないか調べる
    for (const auto d : Dir::ENWS()) {
      if (maze.isWall(focus, d))
        continue; //< 壁があったら更新はしない
      if (known_only && !maze.isKnown(focus, d))
        continue; //< known_only で未知壁なら更新はしない
      if (focus.x > max_x + 1 || focus.y > max_y + 1 || focus.x + 2 < min_x ||
          focus.y + 2 < min_y)
        continue; //< 注目範囲外なら更新しない
      const Vector next = focus.next(d);
      if (getStep(next) <= focus_step + 1)
        continue; //< 更新の必要がない
      setStep(next, focus_step + 1);
      q.push(next); //< 再帰的に更新され得るのでキューにプッシュ
    }
  }
}
const Vector StepMap::calcNextDirs(Maze &maze, const Vectors &dest,
                                   const Vector vec, const Dir dir,
                                   Dirs &nextDirsKnown, Dirs &nextDirCandidates,
                                   const bool prior_unknown) {
  updateSimple(maze, dest, false);
  // 事前に進む候補を決定する
  const auto v = calcNextDirs(maze, vec, dir, nextDirsKnown, nextDirCandidates,
                              prior_unknown);
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
    updateSimple(maze, dest, false);
    calcNextDirs(maze, v, d, tmp_nds, nextDirCandidates, prior_unknown);
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
void StepMap::calcStraightStepTable() {
  const float a = 9000;
  const float v0 = 300;
  const float factor =
      1.0f / (sqrt(pow(v0 / a, 2) + 90 * (MAZE_SIZE * 2) / a) -
              sqrt(pow(v0 / a, 2) + 90 * (MAZE_SIZE * 2 - 1) / a));
  for (int i = 0; i < MAZE_SIZE * 2; ++i) {
    const float x = 90 * (i + 1);
    straightStepTable[i] = (sqrt(pow(v0 / a, 2) + x / a) - v0 / a) * factor;
  }
  for (int i = 0; i < MAZE_SIZE * 2; ++i) {
    const float x = 90 * (i + 1);
    straightStepTable[i] = (sqrt(pow(v0 / a, 2) + x / a) - v0 / a) * factor;
  }
}
const Vector StepMap::calcNextDirs(const Maze &maze, const Vector start_v,
                                   const Dir start_d, Dirs &nextDirsKnown,
                                   Dirs &nextDirCandidates,
                                   const bool prior_unknown) const {
  // ステップマップから既知区間進行方向列を生成
  nextDirsKnown.clear();
  auto focus_v = start_v;
  auto focus_d = start_d;
  while (1) {
    if (maze.unknownCount(focus_v))
      break; //< 未知壁があれば，既知区間は終了
    step_t min_step = MAZE_STEP_MAX;
    // 周囲の区画のうち，最小ステップの方向を求める
    for (const auto d : {focus_d + Dir::Front, focus_d + Dir::Left,
                         focus_d + Dir::Right, focus_d + Dir::Back}) {
      if (maze.isWall(focus_v, d))
        continue; //< 壁があったら行けない
      step_t next_step = getStep(focus_v.next(d));
      if (min_step > next_step) {
        min_step = next_step;
        focus_d = d;
      }
    }
    if (getStep(focus_v) <= min_step)
      break;                          //< 永遠ループ防止
    nextDirsKnown.push_back(focus_d); //< 既知区間移動
    focus_v = focus_v.next(focus_d);  //< 位置を更新
  }
  //< 既知区間終わり
  // ステップマップから未知壁方向の優先順位付方向列を生成
  Dirs dirs;
  // 方向の候補を抽出
  for (const auto d : {focus_d + Dir::Front, focus_d + Dir::Left,
                       focus_d + Dir::Right, focus_d + Dir::Back})
    if (!maze.isWall(focus_v, d) && getStep(focus_v.next(d)) != MAZE_STEP_MAX)
      dirs.push_back(d);
  // ステップが小さい順に並べ替え
  std::sort(dirs.begin(), dirs.end(), [&](const Dir d1, const Dir d2) {
    return getStep(focus_v.next(d1)) <
           getStep(focus_v.next(d2)); //< 低コスト優先
  });
  // 未知壁優先で並べ替え
  if (prior_unknown)
    std::sort(dirs.begin(), dirs.end(),
              [&](const Dir d1 __attribute__((unused)), const Dir d2) {
                return !maze.unknownCount(focus_v.next(d2)); //< 未知壁優先
              });
  // 結果を代入
  nextDirCandidates = dirs;
  return focus_v;
}

} // namespace MazeLib
