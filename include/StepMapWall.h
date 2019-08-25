/**
 * @file StepMapWall.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 壁ベースのステップマップを表現するクラス
 * @date 2019-08-17
 */
#pragma once

#include "Maze.h"
#include "StepMap.h" /*< for step_t, STEP_MAX */

namespace MazeLib {

/**
 * @brief 壁ベースのステップマップを表現するクラス
 */
class StepMapWall {
public:
  StepMapWall();
  void reset(const step_t step = STEP_MAX);
  step_t getStep(const WallIndex i) const {
    return i.isInsideOfFiled() ? step_map[i] : STEP_MAX;
  }
  void setStep(const WallIndex i, const step_t step) {
    if (i.isInsideOfFiled())
      step_map[i] = step;
  }
  void print(const Maze &maze, std::ostream &os = std::cout) const;
  void print(const Maze &maze, const WallIndexes &indexes,
             std::ostream &os = std::cout) const;
  void print(const Maze &maze, const Dirs &shortest_dirs,
             const WallIndex start = WallIndex(0, 0, 1),
             std::ostream &os = std::cout) const;
  void update(const Maze &maze, const WallIndexes &dest, const bool known_only,
              const bool simple);
  bool calcShortestDirs(const Maze &maze, Dirs &shortest_dirs,
                        const bool known_only, const bool simple);
  static const WallIndexes convertDestinations(const Maze &maze,
                                               const Vectors vectors);
  static const Dir convertDir(const Dir d, const WallIndex i);
  static const Dirs convertWallIndexDirsToVectorDirs(const Dirs src,
                                                     const WallIndex start);

private:
  step_t step_map[WallIndex::SIZE]; /**< @brief ステップ数*/
  step_t step_table_along[MAZE_SIZE * 2]; /**< @brief 加速を考慮したステップ */
  step_t step_table_diag[MAZE_SIZE * 2]; /**< @brief 加速を考慮したステップ */

  void calcStraightStepTable();
  const Dirs calcDirsStepDown(const Maze &maze, const WallIndex start,
                              WallIndex &focus, const bool break_unknown,
                              const bool known_only) const;
  const Dirs calcNextDirCandidates(const Maze &maze, const WallIndex focus,
                                   bool prior_unknown) const;
};

} // namespace MazeLib
