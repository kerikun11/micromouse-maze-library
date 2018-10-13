/**
 *  @file SearchAlgorithm.h
 *  @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
 *  @author KERI (Github: kerikun11)
 *  @url https://kerikeri.top/
 *  @date 2017.11.05
 */
#pragma once

#include "Maze.h"
#include "StepMap.h"

namespace MazeLib {
/** @class SearchAlgorithm
 *  @brief 迷路探索アルゴリズムを司るクラス
 */
class SearchAlgorithm {
public:
  /** @brief コンストラクタ
   *  @param maze 使用する迷路の参照
   *  @param goals ゴール区画の配列
   */
  SearchAlgorithm(Maze &maze) : maze(maze) {}
  /** @enum Status
   *  @brief 進むべき方向の計算結果
   */
  enum Status {
    Processing,
    Reached,
    Error,
  };
  /** @enum State
   *  @brief 探索状態を列挙
   */
  enum State {
    START,                  //< 初期位置，初期姿勢
    SEARCHING_FOR_GOAL,     //< ゴール区画を探索中
    SEARCHING_ADDITIONALLY, //< 追加探索中
    BACKING_TO_START,       //< スタートに戻っている
    REACHED_START,          //< スタートに戻ってきた
    IMPOSSIBLE, //< ゴールにだどりつくことができないと判明した
    IDENTIFYING_POSITION, //< 自己位置同定中
    GOING_TO_GOAL,        //< ゴールへ向かっている
  };
  /** @function stateString
   *  @brief Stateの表示用文字列を返す関数
   */
  static const char *stateString(const enum State s);
  /** @function isComplete
   *  @brief 最短経路が導出されているか調べる関数
   */
  bool isComplete();
  void positionIdentifyingInit();
  bool updateWall(const State &state, const Vector &v, const Dir &d,
                  const bool left, const bool front, const bool right,
                  const bool back);
  bool updateWall(const State &state, const Vector &v, const Dir &d,
                  const bool &b);
  bool resetLastWall(const State &state, const int num = 1);
  enum Status calcNextDirs(State &state, Vector &curVec, Dir &curDir,
                           Dirs &nextDirs, Dirs &nextDirCandidates,
                           bool &isPositionIdentifying,
                           bool &isForceBackToStart, bool &isForceGoingToGoal,
                           int &matchCount);
  bool findNextDir(const State state, const Vector v, const Dir d,
                   const Dirs &nextDirCandidates, Dir &nextDir) const;
  bool findNextDir(const Maze &maze, const Vector v, const Dir d,
                   const Dirs &nextDirCandidates, Dir &nextDir) const;
  bool calcShortestDirs(Dirs &shortestDirs, const bool diagonal = true);
  void printMap(const State state, const Vector vec, const Dir dir) const;
  const StepMap &getStepMap() const { return stepMap; }
  const Maze &getMaze() const { return maze; }
  const Vector &getIdStartVector() const { return idStartVector; }
  void setIdStartVector(const Vector &v) { idStartVector = v; }

protected:
  Maze &maze;      /**< 使用する迷路の参照 */
  StepMap stepMap; /**< 使用するステップマップ */

private:
  Maze idMaze; /**< 自己位置同定に使用する迷路 */
  Vector idStartVector;

  /** @function findShortestCandidates
   *  @brief ステップマップにより最短経路上になりうる区画を洗い出す
   */
  bool findShortestCandidates(Vectors &candidates);
  int countIdentityCandidates(const WallLogs idWallLogs,
                              std::pair<Vector, Dir> &ans) const;
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
