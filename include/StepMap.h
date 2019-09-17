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

/**
 * @brief 足立法のためのステップマップを管理するクラス
 */
class StepMap {
public:
  using step_t = uint16_t; /**< @brief ステップの型 */
  static constexpr step_t STEP_MAX =
      std::numeric_limits<step_t>::max(); /**< @brief 最大ステップ値 */

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
   * @param p 区画の座標
   * @return ステップ
   */
  step_t getStep(const int8_t x, const int8_t y) const;
  step_t getStep(const Position p) const { return getStep(p.x, p.y); }
  /**
   * @param ステップへの参照の取得，書き込み可能
   * @param p 区画の座標
   */
  void setStep(const int8_t x, const int8_t y, const step_t step);
  void setStep(const Position p, const step_t step) {
    return setStep(p.x, p.y, step);
  }
  /**
   * @brief ステップの表示
   * @param p ハイライト区画
   */
  void print(const Maze &maze, const Position p = Position(-1, -1),
             const Direction d = Direction::Max,
             std::ostream &os = std::cout) const;
  void print(const Maze &maze, const Directions &dirs,
             const Position start = Position(0, 0),
             std::ostream &os = std::cout) const;
  void printFull(const Maze &maze, const Position p = Position(-1, -1),
                 const Direction d = Direction::Max,
                 std::ostream &os = std::cout) const;
  void printFull(const Maze &maze, const Directions &dirs,
                 const Position start = Position(0, 0),
                 std::ostream &os = std::cout) const;
  /**
   * @brief ステップマップの更新
   * @param dest ステップを0とする区画の配列
   * @param known_only
   *        true:未知の壁は通過不可能とする，false:未知の壁はないものとする
   */
  void update(const Maze &maze, const Positions &dest, const bool known_only,
              const bool simple);
  /**
   * @brief ステップマップから次に行くべき方向を計算する関数
   * @return 既知区間の最終区画
   */
  const Position calcNextDirections(const Maze &maze, const Position start_p,
                                    const Direction start_d,
                                    Directions &nextDirectionsKnown,
                                    Directions &nextDirectionCandidates) const;
  /**
   * @brief 与えられた区画間の最短経路を導出する関数
   * @param maze 迷路の参照
   * @param start 始点区画
   * @param goals ゴール区画の集合
   * @param known_only 既知壁のみモードかどうか
   * @return const Directions スタートからゴールへの最短経路の方向列．
   *                    経路がない場合は空配列となる．
   */
  const Directions calcShortestDirections(const Maze &maze,
                                          const Position start,
                                          const Positions &dest,
                                          const bool known_only,
                                          const bool simple = true);
  /**
   * @brief スタートからゴールまでの最短経路を導出する関数
   */
  const Directions calcShortestDirections(const Maze &maze,
                                          const bool known_only,
                                          const bool simple) {
    return calcShortestDirections(maze, maze.getStart(), maze.getGoals(),
                                  known_only, simple);
  }
  /**
   * @brief ステップマップにより次に行くべき方向列を生成する
   */
  const Directions calcNextDirectionsStepDown(const Maze &maze,
                                              const Pose start, Pose &focus,
                                              const bool known_only,
                                              const bool break_unknown) const;
  /**
   * @brief 引数区画の周囲の未知壁の確認優先順位を生成する関数
   * @return const Directions 確認すべき優先順位
   */
  const Directions calcNextDirectionCandidates(const Maze &maze,
                                               const Pose focus) const;

private:
  step_t step_map[MAZE_SIZE][MAZE_SIZE]; /**< @brief ステップ数 */
  step_t step_table[MAZE_SIZE]; /**< @brief 加速を考慮したステップテーブル */

  /**
   * @brief 最短経路導出用の加速を考慮したステップリストを算出する関数
   * 高速化のため，あらかじめ計算を終えておく．
   */
  void calcStraightStepTable();
};

} // namespace MazeLib
