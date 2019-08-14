/**
 *  @file StepMap.h
 *  @brief マイクロマウスの迷路のステップマップを扱うクラス
 *  @author KERI (Github: kerikun11)
 *  @url https://kerikeri.top/
 *  @date 2017.11.05
 */
#pragma once

#include "Maze.h"

#include <algorithm>

namespace MazeLib {

using step_t = uint16_t; /**< @brief ステップマップの型*/
constexpr step_t MAZE_STEP_MAX = 65000; /**< @brief 最大ステップ値 */

/**
 *  @brief 足立法のためのステップマップを管理するクラス
 */
class StepMap {
public:
  /**
   * @brief Construct a new Step Map object
   */
  StepMap();
  /**
   * @brief ステップマップを初期化する関数
   * @param step この値で初期化する
   */
  void reset(const step_t step = MAZE_STEP_MAX);
  /**
   *  @param ステップへの参照の取得，書き込み可能
   *  @param v 区画の座標
   *  @return ステップメモリの参照
   */
  step_t getStep(const Vector v) const { return getStep(v.x, v.y); }
  step_t getStep(const int8_t x, const int8_t y) const;
  /**
   *  @param ステップへの参照の取得，書き込み可能
   *  @param v 区画の座標
   *  @return ステップメモリの参照
   */
  bool setStep(const Vector v, const step_t step) {
    return setStep(v.x, v.y, step);
  }
  bool setStep(const int8_t x, const int8_t y, const step_t step);
  /**
   *  @param v ハイライト区画
   */
  void print(const Maze &maze, const Vector v = Vector(-1, -1),
             const Dir &d = Dir::Max) const {
    print(std::cout, maze, v, d);
  }
  void print(std::ostream &os, const Maze &maze,
             const Vector v = Vector(-1, -1), const Dir d = Dir::Max) const;
  /** @function update
   *  @brief ステップマップの更新
   *  @param dest ステップを0とする区画の配列
   *  @param known_only
   * true:未知の壁は通過不可能とする，false:未知の壁はないものとする
   */
  void update(const Maze &maze, const Vectors &dest, const bool known_only);
  /** @function calcNextDirs
   *  @brief ステップマップから次に行くべき方向を計算
   */
  const Vector calcNextDirs(Maze &maze, const Vectors &dest, const Vector vec,
                            const Dir dir, Dirs &nextDirsKnown,
                            Dirs &nextDirCandidates,
                            const bool prior_unknown = true);
  bool calcShortestDirs(const Maze &maze, Dirs &shortestDirs,
                        const bool known_only) {
    /* 目的地を作成 */
    update(maze, maze.getGoals(), known_only);
    VecDir end;
    shortestDirs = calcNextDirsKnown(maze, {maze.getStart(), Dir::North}, end,
                                     false, known_only);
    if (getStep(end.first) == 0)
      return true;
    return false;
  }

private:
  step_t stepMap[MAZE_SIZE][MAZE_SIZE]; /**< @brief ステップ数*/
  step_t step_table_along[MAZE_SIZE * 2]; //**< @brief 加速を考慮したステップ */

  /** @function calcStraightStepTable
   *  @brief 最短経路導出用の加速を考慮したステップリストを算出する関数
   *  高速化のため，あらかじめ計算を終えておく．
   */
  void calcStraightStepTable();
  /** @function calcNextDirs
   *  @brief ステップマップにより次に行くべき方向列を生成する
   *  @return true:成功, false:失敗(迷子)
   */
  const Vector calcNextDirs(const Maze &maze, const Vector start_v,
                            const Dir start_d, Dirs &nextDirsKnown,
                            Dirs &nextDirCandidates,
                            const bool prior_unknown) const;
  const Dirs calcNextDirsKnown(const Maze &maze, const VecDir start,
                               VecDir &focus, const bool break_unknown,
                               const bool known_only) const {
    /* ステップマップから既知区間進行方向列を生成 */
    Dirs nextDirsKnown;
    /* start から順にステップマップを下って行く */
    focus = start;
    while (1) {
      /* 周辺の走査; 未知壁の有無と，最小ステップの方向を求める */
      auto min_d = Dir::Max;
      auto min_step = MAZE_STEP_MAX;
      for (const auto d :
           {focus.second + Dir::Front, focus.second + Dir::Left,
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
  const Dirs calcNextDirCandidates(const Maze &maze, const VecDir focus,
                                   bool prior_unknown) const {
    /* 方向の候補を抽出 */
    Dirs dirs;
    for (const auto d : {focus.second + Dir::Front, focus.second + Dir::Left,
                         focus.second + Dir::Right, focus.second + Dir::Back}) {
      const auto next = focus.first.next(d);
      if (!maze.isWall(focus.first, d) && getStep(next) != MAZE_STEP_MAX)
        dirs.push_back(d);
    }
    /* ステップが小さい順に並べ替え */
    std::sort(dirs.begin(), dirs.end(), [&](const Dir d1, const Dir d2) {
      return getStep(focus.first.next(d1)) <
             getStep(focus.first.next(d2)); /*< 低コスト優先 */
    });
    /* 未知壁優先で並べ替え, これがないと探索時間増大 */
    if (prior_unknown)
      std::sort(dirs.begin(), dirs.end(),
                [&](const Dir d1 __attribute__((unused)), const Dir d2) {
                  return !maze.unknownCount(focus.first.next(d2));
                });
    return dirs;
  }
};

} // namespace MazeLib
