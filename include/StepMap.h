/**
 *  @file StepMap.h
 *  @brief マイクロマウスの迷路のステップマップを扱うクラス
 *  @author KERI (Github: kerikun11)
 *  @url https://kerikeri.top/
 *  @date 2017.11.05
 */
#pragma once

#include "Maze.h"

namespace MazeLib {
typedef uint16_t step_t;               /**< @brief ステップマップの型*/
constexpr step_t MAZE_STEP_MAX = 9999; /**< @brief 最大ステップ値 */

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
             const Dir &d = Dir::AbsMax) const {
    print(std::cout, maze, v, d);
  }
  void print(std::ostream &os, const Maze &maze,
             const Vector v = Vector(-1, -1), const Dir d = Dir::AbsMax) const;
  /** @function update
   *  @brief ステップマップの更新
   *  @param dest ステップを0とする区画の配列
   *  @param known_only
   * true:未知の壁は通過不可能とする，false:未知の壁はないものとする
   *  @param diag_enabled true: 斜め直線あり false: 斜めはジグザグ
   */
  void update(const Maze &maze, const Vectors &dest,
              const bool known_only = false, const bool diag_enabled = true);
  /** @function updateSimple
   *  @brief シンプルに1マス1ステップのステップマップ更新
   *  @param dest ステップを0とする区画の配列(destination)
   *  @param known_only
   * true:未知の壁は通過不可能とする，false:未知の壁はないものとする
   */
  void updateSimple(const Maze &maze, const Vectors &dest,
                    const bool known_only = false);
  /** @function calcNextDirs
   *  @brief ステップマップから次に行くべき方向を計算
   */
  const Vector calcNextDirs(Maze &maze, const Vectors &dest, const Vector vec,
                            const Dir dir, Dirs &nextDirsKnown,
                            Dirs &nextDirCandidates);

private:
  step_t stepMap[MAZE_SIZE][MAZE_SIZE]; /**< @brief ステップ数*/
  step_t straightStepTable[MAZE_SIZE *
                           2]; //**< @brief
                               //最短経路導出用の加速を考慮したステップリスト*/

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
                            Dirs &nextDirCandidates) const;
};
} // namespace MazeLib
