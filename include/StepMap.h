/**
 * @file StepMap.h
 * @brief マイクロマウスの迷路のステップマップを扱うクラス
 * @author KERI (Github: kerikun11)
 * @url https://kerikeri.top/
 * @date 2017.11.05
 */
#pragma once

#include "Maze.h"
#include <limits> /*< for std::numeric_limits */

namespace MazeLib {

using step_t = uint16_t; /**< @brief ステップの型 */
static constexpr step_t STEP_MAX =
    std::numeric_limits<step_t>::max(); /**< @brief 最大ステップ値 */

/**
 * @brief 足立法のためのステップマップを管理するクラス
 */
class StepMap {
public:
  /**
   * @brief コンストラクタ
   */
  StepMap();
  /**
   * @brief ステップマップを初期化する関数
   * @param step この値で初期化する
   */
  void reset(const step_t step = STEP_MAX);
  /**
   * @param ステップの取得
   * @param v 区画の座標
   * @return ステップ
   */
  step_t getStep(const int8_t x, const int8_t y) const;
  step_t getStep(const Vector v) const { return getStep(v.x, v.y); }
  /**
   * @param ステップへの参照の取得，書き込み可能
   * @param v 区画の座標
   */
  void setStep(const int8_t x, const int8_t y, const step_t step);
  void setStep(const Vector v, const step_t step) {
    return setStep(v.x, v.y, step);
  }
  /**
   * @brief ステップの表示
   * @param v ハイライト区画
   */
  void print(const Maze &maze, const Vector v = Vector(-1, -1),
             const Dir d = Dir::Max, std::ostream &os = std::cout) const;
  void print(const Maze &maze, const Dirs &dirs,
             const Vector start = Vector(0, 0),
             std::ostream &os = std::cout) const;
  void printFull(const Maze &maze, std::ostream &os = std::cout) const;
  /**
   * @brief ステップマップの更新
   * @param dest ステップを0とする区画の配列
   * @param known_only
   *        true:未知の壁は通過不可能とする，false:未知の壁はないものとする
   */
  void update(const Maze &maze, const Vectors &dest, const bool known_only,
              const bool simple);
  /**
   * @brief ステップマップから次に行くべき方向を計算する関数
   * @return 既知区間の最終区画
   */
  const Vector calcNextDirs(const Maze &maze, const Vector start_v,
                            const Dir start_d, Dirs &nextDirsKnown,
                            Dirs &nextDirCandidates) const;
  /**
   * @brief 迷路を編集してさらに優先順の精度を向上させる関数
   * @return 既知区間の最終区画
   */
  const Vector calcNextDirsAdv(Maze &maze, const Vectors &dest,
                               const Vector vec, const Dir dir,
                               Dirs &nextDirsKnown, Dirs &nextDirCandidates);
  /**
   * @brief 与えられた区画間の最短経路を導出する関数
   * @param maze 迷路の参照
   * @param start 始点区画
   * @param goals ゴール区画の集合
   * @param known_only 既知壁のみモードかどうか
   * @return const Dirs スタートからゴールへの最短経路の方向列．
   *                    経路がない場合は空配列となる．
   */
  const Dirs calcShortestDirs(const Maze &maze, const Vector start,
                              const Vectors &dest, const bool known_only,
                              const bool simple);
  /**
   * @brief スタートからゴールまでの最短経路を導出する関数
   */
  const Dirs calcShortestDirs(const Maze &maze, const bool known_only,
                              const bool simple) {
    return calcShortestDirs(maze, maze.getStart(), maze.getGoals(), known_only,
                            simple);
  }

private:
  step_t step_map[MAZE_SIZE][MAZE_SIZE]; /**< @brief ステップ数 */
  step_t step_table_along[MAZE_SIZE * 2]; /**< @brief 加速を考慮したステップ */

  /**
   * @brief 最短経路導出用の加速を考慮したステップリストを算出する関数
   * 高速化のため，あらかじめ計算を終えておく．
   */
  void calcStraightStepTable();
  /**
   * @brief ステップマップにより次に行くべき方向列を生成する
   */
  const Dirs calcNextDirsStepDown(const Maze &maze, const VecDir start,
                                  VecDir &focus, const bool known_only,
                                  const bool break_unknown) const;
  /**
   * @brief 引数区画の周囲の未知壁の確認優先順位を生成する関数
   * @return const Dirs 確認すべき優先順位
   */
  const Dirs calcNextDirCandidates(const Maze &maze, const VecDir focus) const;
};

} // namespace MazeLib
