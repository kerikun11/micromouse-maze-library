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
  enum Result : uint8_t {
    Processing, /**< 現探索状態を継続 */
    Reached,    /**< 現探索状態が完了 */
    Error,      /**< エラー */
  };
  /**
   *  @brief 探索状態を列挙
   */
  enum State : uint8_t {
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
  void positionIdentifyingInit(Vector *pVector, Dir *pDir, const Dir estIniDir);
  bool updateWall(const State state, const Vector v, const Dir d,
                  const bool left, const bool front, const bool right,
                  const bool back);
  bool updateWall(const State state, const Vector v, const Dir d, const bool b);
  void resetLastWall(const State state, const int num = 1);
  enum Result calcNextDirs(State &state, Vector &curVec, Dir &curDir,
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
  const Dirs candidatesIncludeStart(const Vector cv) {
    Dirs result_dirs;
    const auto offset_v = Vector(0, 1) - (cv - idOffset);
    const int min_diff = 4;
    for (const auto offset_d : Dir::ENWS()) {
      int diffs = 0;
      int unknown = 0;
      for (const auto wl : idMaze.getWallLogs()) {
        const auto maze_v = (Vector(wl) - idOffset).rotate(offset_d) + offset_v;
        const auto maze_d = wl.d + offset_d;
        if (maze_v.isOutsideofField())
          break;
        if (maze.isKnown(maze_v, maze_d) && maze.isWall(maze_v, maze_d) != wl.b)
          diffs++;
        if (!maze.isKnown(maze_v, maze_d))
          unknown++;
        /* 打ち切り */
        if (diffs > min_diff)
          break;
      }
      /* 非一致条件 */
      if (diffs > min_diff ||
          unknown * 4 > (int)idMaze.getWallLogs().size() * 3)
        continue;
      // const auto cv_cand = (cv - idOffset).rotate(offset_d) + offset_v;
      // const auto cd_cand = cd + offset_d;
      // logi << "start cand.   " << offset_v << std::endl;
      result_dirs.push_back(offset_d);
    }
    return result_dirs;
  }

protected:
  Maze &maze;                          /**< 使用する迷路の参照 */
  StepMap stepMap;                     /**< 使用するステップマップ */
  ShortestAlgorithm shortestAlgorithm; /**< 最短経路導出器 */

private:
  Maze idMaze;     /**< 自己位置同定に使用する迷路 */
  Vector idOffset; /**< 自己位置同定迷路の始点位置 */
  Dir estIniDir;

  /**
   *  @brief ステップマップにより最短経路上になりうる区画を洗い出す
   */
  bool findShortestCandidates(Vectors &candidates);
  int countIdentityCandidates(const WallLogs &idWallLogs, VecDir &ans) const;
  /**
   * @brief 各状態での進行方向列導出関数
   */
  enum Result calcNextDirsSearchForGoal(const Vector cv, const Dir cd,
                                        Dirs &nextDirsKnown,
                                        Dirs &nextDirCandidates);
  enum Result calcNextDirsSearchAdditionally(const Vector cv, const Dir cd,
                                             Dirs &nextDirsKnown,
                                             Dirs &nextDirCandidates);
  enum Result calcNextDirsBackingToStart(const Vector cv, const Dir cd,
                                         Dirs &nextDirsKnown,
                                         Dirs &nextDirCandidates);
  enum Result calcNextDirsGoingToGoal(const Vector cv, const Dir cd,
                                      Dirs &nextDirsKnown,
                                      Dirs &nextDirCandidates);
  enum Result calcNextDirsPositionIdentification(Vector &cv, Dir &cd,
                                                 Dirs &nextDirsKnown,
                                                 Dirs &nextDirCandidates,
                                                 int &matchCount);
};

} // namespace MazeLib
