/**
 * @file StepMap.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief マイクロマウスの迷路のステップマップを扱うクラス
 * @date 2017-11-05
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
  void reset(const step_t step = STEP_MAX) { step_map.fill(step); }
  /**
   * @brief ステップの取得
   * @details 盤面外なら `STEP_MAX` を返す
   */
  step_t getStep(const int8_t x, const int8_t y) const {
    return getStep(Position(x, y));
  }
  step_t getStep(const Position p) const {
    return p.isInsideOfField() ? step_map[p.getIndex()] : STEP_MAX;
  }
  /**
   * @brief ステップの更新
   * @details 盤面外なら何もしない
   */
  void setStep(const int8_t x, const int8_t y, const step_t step) {
    return setStep(Position(x, y), step);
  }
  void setStep(const Position p, const step_t step) {
    if (p.isInsideOfField())
      step_map[p.getIndex()] = step;
  }
  /**
   * @brief ステップマップの生配列への参照を取得
   */
  const auto &getMapArray() const { return step_map; }
  /**
   * @brief ステップの表示
   * @param p ハイライト区画
   */
  void print(const Maze &maze, const Position &p = Position(-1, -1),
             const Direction d = Direction::Max,
             std::ostream &os = std::cout) const;
  void print(const Maze &maze, const Directions &dirs,
             const Position &start = Position(0, 0),
             std::ostream &os = std::cout) const;
  void printFull(const Maze &maze, const Position &p = Position(-1, -1),
                 const Direction d = Direction::Max,
                 std::ostream &os = std::cout) const;
  void printFull(const Maze &maze, const Directions &dirs,
                 const Position &start = Position(0, 0),
                 std::ostream &os = std::cout) const;
  /**
   * @brief ステップマップの更新
   * @param dest ステップを0とする目的地の区画の集合(順不同)
   * @param known_only true:未知壁は通過不可能，false:未知壁は通過可能とする
   * @param simple 台形加速を考慮せず，隣接区画のコストをすべて1にする
   */
  void update(const Maze &maze, const Positions &dest, const bool known_only,
              const bool simple);
  /**
   * @brief 与えられた区画間の最短経路を導出する関数
   * @param maze 迷路の参照
   * @param start 始点区画
   * @param dest 目的地区画の集合(順不同)
   * @param known_only 既知壁のみモードかどうか
   * @return const Directions スタートからゴールへの最短経路の方向列．
   *                    経路がない場合は空配列となる．
   */
  Directions calcShortestDirections(const Maze &maze, const Position &start,
                                    const Positions &dest,
                                    const bool known_only, const bool simple);
  /**
   * @brief スタートからゴールまでの最短経路を導出する関数
   */
  Directions calcShortestDirections(const Maze &maze, const bool known_only,
                                    const bool simple) {
    return calcShortestDirections(maze, maze.getStart(), maze.getGoals(),
                                  known_only, simple);
  }
  /**
   * @brief ステップマップから次に行くべき方向を計算する関数
   * @return 既知区間の最終区画
   */
  Pose calcNextDirections(const Maze &maze, const Pose &start,
                          Directions &nextDirectionsKnown,
                          Directions &nextDirectionCandidates) const;
  /**
   * @brief ステップマップにより次に行くべき方向列を生成する
   */
  Directions getStepDownDirections(const Maze &maze, const Pose &start,
                                   Pose &end, const bool known_only,
                                   const bool break_unknown) const;
  /**
   * @brief 引数区画の周囲の未知壁の確認優先順位を生成する関数
   * @return const Directions 行くべき方向の優先順位
   */
  Directions getNextDirectionCandidates(const Maze &maze,
                                        const Pose &focus) const;
  /**
   * @brief ゴール区画内を行けるところまで直進させる方向列を追加する関数
   * @param maze 迷路の参照
   * @param shortest_dirs 追記元の方向列．これ自体に追記される．
   * @param diag_enabled 斜めありなし
   */
  static void appendStraightDirections(const Maze &maze,
                                       Directions &shortest_dirs,
                                       const bool known_only,
                                       const bool diag_enabled);

protected:
  std::array<step_t, Position::SIZE> step_map; /**< @brief ステップ数*/
  /** @brief 台形加速を考慮したコストテーブル (壁沿い) */
  std::array<step_t, MAZE_SIZE> step_table;

  /**
   * @brief 最短経路導出用の加速を考慮したステップリストを算出する関数
   * 高速化のため，あらかじめ計算を終えておく．
   */
  void calcStraightStepTable();
};

} // namespace MazeLib
