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
  StepMapWall() { calcStraightStepTable(); }
  step_t getStep(const WallIndex i) const {
    return i.isInsideOfField() ? step_map[i.getIndex()] : STEP_MAX;
  }
  void setStep(const WallIndex i, const step_t step) {
    if (i.isInsideOfField())
      step_map[i.getIndex()] = step;
  }
  /**
   * @brief ステップマップの表示
   */
  void print(const Maze& maze,
             const WallIndexes& indexes,
             const bool show_full_step = false,
             std::ostream& os = std::cout) const;
  void print(const Maze& maze,
             const Directions& shortest_dirs,
             const WallIndex& start = START_WALL_INDEX,
             const bool show_full_step = false,
             std::ostream& os = std::cout) const;
  /**
   * @brief 迷路上にパスを表示
   */
  void printPath(const Maze& maze,
                 const WallIndexes& indexes,
                 std::ostream& os = std::cout) const;
  void printPath(const Maze& maze,
                 const Directions& shortest_dirs,
                 const WallIndex& start = START_WALL_INDEX,
                 std::ostream& os = std::cout) const;
  void update(const Maze& maze,
              const WallIndexes& dest,
              const bool known_only,
              const bool simple);
  Directions calcShortestDirections(const Maze& maze,
                                    const bool known_only,
                                    const bool simple) {
    return calcShortestDirections(maze, START_WALL_INDEX,
                                  convertDestinations(maze, maze.getGoals()),
                                  known_only, simple);
  }
  Directions calcShortestDirections(const Maze& maze,
                                    const WallIndex& start,
                                    const WallIndexes& dest,
                                    const bool known_only,
                                    const bool simple);
  Directions getStepDownDirections(const Maze& maze,
                                   const WallIndex& start,
                                   WallIndex& end,
                                   const bool known_only,
                                   const bool break_unknown) const;
  static WallIndexes convertDestinations(const Maze& maze,
                                         const Positions& positions);
  static Direction convertWallIndexDirection(const WallIndex& i,
                                             const Direction d);
  static Directions convertWallIndexDirectionsToPositionDirections(
      const Directions& src);
  static void appendStraightDirections(
      const Maze& maze,
      Directions& shortest_dirs,
      const WallIndex& start = START_WALL_INDEX);

 private:
  /** @brief 迷路中のステップ数 */
  std::array<step_t, WallIndex::SIZE> step_map;
  /** @brief 台形加速を考慮した移動コストテーブル (壁沿い方向) */
  std::array<step_t, MAZE_SIZE * 2> step_table_along;
  /** @brief 台形加速を考慮した移動コストテーブル (斜め方向) */
  std::array<step_t, MAZE_SIZE * 2> step_table_diag;

  /**
   * @brief 計算の高速化のために予め直進のコストテーブルを生成する関数
   */
  void calcStraightStepTable();
};

}  // namespace MazeLib
