/**
 * @file StepMapWall.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 壁ベースのステップマップを表現するクラス
 * @date 2019-08-17
 */
#pragma once

#include "Maze.h"
#include <limits> /*< for std::numeric_limits */

namespace MazeLib {

/**
 * @brief 壁ベースのステップマップを表現するクラス
 */
class StepMapWall {
public:
  using step_t = uint16_t; /**< @brief ステップの型 */
  static constexpr step_t STEP_MAX =
      std::numeric_limits<step_t>::max(); /**< @brief 最大ステップ値 */

public:
  StepMapWall() { calcStraightStepTable(); }
  void print(const Maze &maze, std::ostream &os = std::cout) const;
  void print(const Maze &maze, const WallIndexes &indexes,
             std::ostream &os = std::cout) const;
  void print(const Maze &maze, const Directions &shortest_dirs,
             const WallIndex &start = WallIndex(0, 0, 1),
             std::ostream &os = std::cout) const;
  void update(const Maze &maze, const WallIndexes &dest, const bool known_only,
              const bool simple);
  const Directions calcShortestDirections(const Maze &maze,
                                          const bool known_only,
                                          const bool simple) {
    return calcShortestDirections(maze, WallIndex(0, 0, 1),
                                  convertDestinations(maze, maze.getGoals()),
                                  known_only, simple);
  }
  const Directions calcShortestDirections(const Maze &maze,
                                          const WallIndex &start,
                                          const WallIndexes &dest,
                                          const bool known_only,
                                          const bool simple);
  static const WallIndexes convertDestinations(const Maze &maze,
                                               const Positions &positions);
  static const Direction convertDirection(const Direction d,
                                          const WallIndex &i);
  static const Directions
  convertWallIndexDirectionsToPositionDirections(const Directions &src,
                                                 const WallIndex &start);
  static void appendStraightDirections(const Maze &maze,
                                       Directions &shortest_dirs);

private:
  std::array<step_t, WallIndex::SIZE> step_map; /**< @brief ステップ数*/
  std::array<step_t, MAZE_SIZE * 2>
      step_table_along; /*< @brief 台形加速を考慮したコストテーブル */
  std::array<step_t, MAZE_SIZE * 2>
      step_table_diag; /*< @brief 台形加速を考慮したコストテーブル(斜め) */

  void calcStraightStepTable();
};

} // namespace MazeLib
