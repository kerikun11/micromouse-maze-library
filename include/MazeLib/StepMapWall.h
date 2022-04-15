/**
 * @file StepMapWall.h
 * @brief 壁ベースのステップマップを表現するクラス
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-08-17
 * @copyright Copyright 2019 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include "MazeLib/Maze.h"

#include <limits> /*< for std::numeric_limits */

namespace MazeLib {

/**
 * @brief 壁ベースのステップマップを表現するクラス
 */
class StepMapWall {
 public:
  using step_t = uint16_t; /**< @brief ステップの型 */
  /**
   * @brief 最大ステップ値
   */
  static constexpr step_t STEP_MAX = std::numeric_limits<step_t>::max();
  /**
   * @brief スタート区画の WallIndex
   */
  static const WallIndex START_WALL_INDEX;

 public:
  StepMapWall() { calcStraightCostTable(); }
  /**
   * @brief ステップを取得する
   * @details 盤面外なら `STEP_MAX` を返す
   */
  step_t getStep(const WallIndex i) const {
    return i.isInsideOfField() ? stepMap[i.getIndex()] : STEP_MAX;
  }
  /**
   * @brief ステップを更新する
   * @details 盤面外なら何もしない
   */
  void setStep(const WallIndex i, const step_t step) {
    if (i.isInsideOfField())
      stepMap[i.getIndex()] = step;
  }
  /**
   * @brief ステップマップの表示
   */
  void print(const Maze& maze,
             const WallIndexes& indexes,
             const bool showFullStep = false,
             std::ostream& os = std::cout) const;
  void print(const Maze& maze,
             const Directions& shortestDirections,
             const WallIndex& start = START_WALL_INDEX,
             const bool showFullStep = false,
             std::ostream& os = std::cout) const;
  /**
   * @brief 迷路上に WallIndex パスを表示
   */
  void printPath(const Maze& maze,
                 const WallIndexes& indexes,
                 std::ostream& os = std::cout) const;
  void printPath(const Maze& maze,
                 const Directions& shortestDirections,
                 const WallIndex& start = START_WALL_INDEX,
                 std::ostream& os = std::cout) const;
  /**
   * @brief ステップマップの更新
   *
   * @param maze 使用する迷路情報
   * @param dest 目的地の集合
   * @param knownOnly 既知壁のみで更新(未知壁は壁ありとみなす)
   * @param simple 台形加速を考慮しないシンプルな更新
   */
  void update(const Maze& maze,
              const WallIndexes& dest,
              const bool knownOnly,
              const bool simple);
  Directions calcShortestDirections(const Maze& maze,
                                    const bool knownOnly,
                                    const bool simple) {
    return calcShortestDirections(maze, START_WALL_INDEX,
                                  convertDestinations(maze, maze.getGoals()),
                                  knownOnly, simple);
  }
  Directions calcShortestDirections(const Maze& maze,
                                    const WallIndex& start,
                                    const WallIndexes& dest,
                                    const bool knownOnly,
                                    const bool simple);
  Directions getStepDownDirections(const Maze& maze,
                                   const WallIndex& start,
                                   WallIndex& end,
                                   const bool knownOnly,
                                   const bool simple,
                                   const bool breakUnknown) const;
  static WallIndexes convertDestinations(const Maze& maze,
                                         const Positions& positions);
  static Direction convertWallIndexDirection(const WallIndex& i,
                                             const Direction d);
  static Directions convertWallIndexDirectionsToPositionDirections(
      const Directions& src);
  static void appendStraightDirections(
      const Maze& maze,
      Directions& shortestDirections,
      const WallIndex& start = START_WALL_INDEX);

#if MAZE_DEBUG_PROFILING
  std::size_t queueSizeMax = 0;
#endif

 private:
  /** @brief 迷路中のステップ数 */
  std::array<step_t, WallIndex::SIZE> stepMap;
  /** @brief コストテーブルのサイズ */
  static constexpr int stepTableSize = MAZE_SIZE * 2;
  /** @brief コストが最大値を超えないようにスケーリングする係数 */
  static constexpr float scalingFactor = 2;
  /** @brief 台形加速を考慮した移動コストテーブル (壁沿い方向) */
  std::array<step_t, stepTableSize> stepTableAlong;
  /** @brief 台形加速を考慮した移動コストテーブル (斜め方向) */
  std::array<step_t, stepTableSize> stepTableDiag;

  /**
   * @brief 計算の高速化のために予め直進のコストテーブルを計算する関数
   */
  void calcStraightCostTable();
};

}  // namespace MazeLib
