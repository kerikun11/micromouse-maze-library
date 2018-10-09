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
/** @constexpr MAZE_STEP_MAX
 *  @brief ステップマップの最大値
 */
constexpr int MAZE_STEP_MAX = 9999;
typedef uint16_t step_t; /**< @brief ステップマップの型*/

/** @class StepMap
 *  @brief 足立法のためのステップマップを管理するクラス
 */
class StepMap {
public:
  /** @brief コンストラクタ
   *  @param maze 使用する迷路の参照
   */
  StepMap();
  /** @function reset
   *  @brief ステップマップの初期化
   */
  void reset(const step_t step = MAZE_STEP_MAX);
  /** @function getStep
   *  @param ステップへの参照の取得，書き込み可能
   *  @param v 区画の座標
   *  @return ステップメモリの参照
   */
  const step_t &getStep(const Vector &v) const { return getStep(v.x, v.y); }
  const step_t &getStep(const int8_t &x, const int8_t &y) const;
  /** @function getStep
   *  @param ステップへの参照の取得，書き込み可能
   *  @param v 区画の座標
   *  @return ステップメモリの参照
   */
  bool setStep(const Vector &v, const step_t &step) {
    return setStep(v.x, v.y, step);
  }
  bool setStep(const int8_t &x, const int8_t &y, const step_t &step);
  /** @function print
   *  @param v ハイライト区画
   */
  void print(const Maze &maze, const Vector &v = Vector(-1, -1),
             const Dir &d = Dir::AbsMax) const {
    print(std::cout, maze, v, d);
  }
  void print(std::ostream &os, const Maze &maze,
             const Vector &v = Vector(-1, -1),
             const Dir &d = Dir::AbsMax) const;
  /** @function update
   *  @brief ステップマップの更新
   *  @param dest ステップを0とする区画の配列
   *  @param onlyCanGo
   * true:未知の壁は通過不可能とする，false:未知の壁はないものとする
   *  @param diagonal true: 斜め直線あり false: 斜めはジグザグ
   */
  void update(const Maze &maze, const Vectors &dest,
              const bool onlyCanGo = false, const bool diagonal = true);
  /** @function updateSimple
   *  @brief シンプルに1マス1ステップのステップマップ更新
   *  @param dest ステップを0とする区画の配列(destination)
   *  @param onlyCanGo
   * true:未知の壁は通過不可能とする，false:未知の壁はないものとする
   */
  void updateSimple(const Maze &maze, const Vectors &dest,
                    const bool onlyCanGo = false);
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
  Vector calcNextDirs(const Maze &maze, const Vector &start_v,
                      const Dir &start_d, Dirs &nextDirsKnown,
                      Dirs &nextDirCandidates) const;
};
} // namespace MazeLib
