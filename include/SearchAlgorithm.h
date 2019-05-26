/**
 *  @file SearchAlgorithm.h
 *  @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
 *  @author KERI (Github: kerikun11)
 *  @url https://kerikeri.top/
 *  @date 2017.11.05
 */
#pragma once

#include "Maze.h"
#include "ShortestAlgorithm.h"
#include "StepMap.h"

namespace MazeLib {
/**
 *  @brief 迷路探索アルゴリズムを司るクラス
 */
class SearchAlgorithm {
public:
  /**
   *  @brief 進むべき方向の計算結果
   */
  enum Status {
    Processing, /**< 現探索状態を継続 */
    Reached,    /**< 現探索状態が完了 */
    Error,      /**< エラー */
  };
  /**
   *  @brief 探索状態を列挙
   */
  enum State {
    START,                  /**< 初期位置，初期姿勢 */
    SEARCHING_FOR_GOAL,     /**< ゴール区画を探索中 */
    SEARCHING_ADDITIONALLY, /**< 追加探索中 */
    BACKING_TO_START,       /**< スタートに戻っている */
    REACHED_START,          /**< スタートに戻ってきた */
    IMPOSSIBLE, /**< ゴールにだどりつくことができないと判明した */
    IDENTIFYING_POSITION, /**< 自己位置同定中 */
    GOING_TO_GOAL,        /**< ゴールへ向かっている */
  };
  /**
   *  @brief Stateの表示用文字列を返す関数
   */
  static const char *stateString(const enum State s);

public:
  SearchAlgorithm(Maze &maze) : maze(maze), shortestAlgorithm(maze) {}
  bool isComplete();
  void positionIdentifyingInit(Vector *pVector, Dir *pDir);
  bool updateWall(const State &state, const Vector &v, const Dir &d,
                  const bool left, const bool front, const bool right,
                  const bool back);
  bool updateWall(const State &state, const Vector &v, const Dir &d,
                  const bool &b);
  void resetLastWall(const State &state, const int num = 1);
  enum Status calcNextDirs(State &state, Vector &curVec, Dir &curDir,
                           Dirs &nextDirs, Dirs &nextDirCandidates,
                           bool &isPositionIdentifying,
                           bool &isForceBackToStart, bool &isForceGoingToGoal,
                           int &matchCount);
  bool findNextDir(const State state, const Vector v, const Dir d,
                   const Dirs &nextDirCandidates, Dir &nextDir) const;
  bool findNextDir(const Maze &maze, const Vector v, const Dir d,
                   const Dirs &nextDirCandidates, Dir &nextDir) const;
  bool calcShortestDirs(Dirs &shortestDirs, const bool diag_enabled = true);
  void printMap(const State state, const Vector vec, const Dir dir) const;
  const StepMap &getStepMap() const { return stepMap; }
  const Maze &getMaze() const { return maze; }
  const ShortestAlgorithm &getShortestAlgorithm() const {
    return shortestAlgorithm;
  }

protected:
  Maze &maze;                          /**< 使用する迷路の参照 */
  StepMap stepMap;                     /**< 使用するステップマップ */
  ShortestAlgorithm shortestAlgorithm; /**< 最短経路導出器 */

private:
  Maze idMaze;     /**< 自己位置同定に使用する迷路 */
  Vector idOffset; /**< 自己位置同定迷路の始点位置 */

  /**
   *  @brief ステップマップにより最短経路上になりうる区画を洗い出す
   */
  bool findShortestCandidates(Vectors &candidates);
  int countIdentityCandidates(const WallLogs &idWallLogs,
                              std::pair<Vector, Dir> &ans) const;
  /**
   * @brief 各状態での進行方向列導出関数
   */
  enum Status calcNextDirsSearchForGoal(const Vector &cv, const Dir &cd,
                                        Dirs &nextDirsKnown,
                                        Dirs &nextDirCandidates);
  enum Status calcNextDirsSearchAdditionally(const Vector &cv, const Dir &cd,
                                             Dirs &nextDirsKnown,
                                             Dirs &nextDirCandidates);
  enum Status calcNextDirsBackingToStart(const Vector &cv, const Dir &cd,
                                         Dirs &nextDirsKnown,
                                         Dirs &nextDirCandidates);
  enum Status calcNextDirsGoingToGoal(const Vector &cv, const Dir &cd,
                                      Dirs &nextDirsKnown,
                                      Dirs &nextDirCandidates);
  enum Status calcNextDirsPositionIdentification(Vector &cv, Dir &cd,
                                                 Dirs &nextDirsKnown,
                                                 Dirs &nextDirCandidates,
                                                 int &matchCount);
};

} // namespace MazeLib
