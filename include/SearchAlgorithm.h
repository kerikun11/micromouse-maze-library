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
#include "StepMapSlalom.h"
#include "StepMapWall.h"

namespace MazeLib {
/**
 * @brief 迷路探索アルゴリズムを司るクラス
 */
class SearchAlgorithm {
public:
  /**
   * @brief 進むべき方向の計算結果
   */
  enum Result : uint8_t {
    Processing, /**< 現探索状態を継続 */
    Reached,    /**< 現探索状態が完了 */
    Error,      /**< エラーが発生 */
  };
  /**
   * @brief 探索状態を列挙
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
   * @brief Stateの表示用文字列を返す関数
   */
  static const char *getStateString(const State s);

public:
  SearchAlgorithm(Maze &maze) : maze(maze) {}
  bool isComplete();
  void positionIdentifyingInit(Pose &current_pose);
  bool updateWall(const State state, const Pose &pose, const bool left,
                  const bool front, const bool right);
  bool updateWall(const State state, const Position &p, const Direction d,
                  const bool b);
  void resetLastWalls(const State state, const int num = 1);
  Result calcNextDirections(State &state, Pose &current_pose,
                            Directions &nextDirections,
                            Directions &nextDirectionCandidates,
                            bool &isPositionIdentifying,
                            bool &isForceBackToStart, bool &isForceGoingToGoal,
                            int &matchCount);
  bool findNextDirection(const State state, const Pose &pose,
                         const Directions &nextDirectionCandidates,
                         Direction &nextDirection) const;
  bool findNextDirection(const Maze &maze, const Pose &pose,
                         const Directions &nextDirectionCandidates,
                         Direction &nextDirection) const;
  bool calcShortestDirections(Directions &shortest_dirs,
                              const bool diag_enabled = true);
  StepMapSlalom::cost_t getShortestCost() const {
    return getStepMapSlalom().getShortestCost();
  }
  void printMap(const State state, const Pose &pose) const;

  /**
   * @brief Getters
   */
  const Maze &getMaze() const { return maze; }
  const StepMap &getStepMap() const { return step_map; }
  const StepMapWall &getStepMapWall() const { return step_map_wall; }
  const StepMapSlalom &getStepMapSlalom() const { return step_map_slalom; }
  const Maze &getIdMaze() const { return idMaze; }

protected:
  Maze &maze;                    /**< 使用する迷路の参照 */
  StepMap step_map;              /**< 使用するステップマップ */
  StepMapWall step_map_wall;     /**< 使用するステップマップ */
  StepMapSlalom step_map_slalom; /**< 使用するステップマップ */

private:
  Maze idMaze;       /**< 自己位置同定に使用する迷路 */
  Position idOffset; /**< 自己位置同定迷路の始点位置 */

  /**
   * @brief ステップマップにより最短経路上になりうる区画を洗い出す
   */
  bool findShortestCandidates(Positions &candidates, const bool simple);
  /**
   * @brief 自己位置同定のパターンにマッチする候補をカウントする
   *
   * @param idWallLogs
   * @param ans マッチした解のひとつ
   * @return int マッチ数, 0: 失敗, 1: 特定, 2-: 複数マッチ
   */
  int countIdentityCandidates(const WallLogs &idWallLogs, Pose &ans) const;
  /**
   * @brief 特定の区画にマッチする方向の候補を探す
   * スタート区画への訪問を避けるために使用する関数
   *
   * @param cur_p 注目する区画
   * @param target 検索対象の区画と方向
   * @return const Directions 注目する区画からの方向列
   */
  const Directions
  findMatchDirectionCandidates(const Position &current_position,
                               const Pose &target) const;

  /**
   * @brief 各状態での進行方向列導出関数
   */
  Result calcNextDirectionsSearchForGoal(const Pose &current_pose,
                                         Directions &nextDirectionsKnown,
                                         Directions &nextDirectionCandidates);
  Result
  calcNextDirectionsSearchAdditionally(const Pose &current_pose,
                                       Directions &nextDirectionsKnown,
                                       Directions &nextDirectionCandidates);
  Result calcNextDirectionsBackingToStart(const Pose &current_pose,
                                          Directions &nextDirectionsKnown,
                                          Directions &nextDirectionCandidates);
  Result calcNextDirectionsGoingToGoal(const Pose &current_pose,
                                       Directions &nextDirectionsKnown,
                                       Directions &nextDirectionCandidates);
  Result calcNextDirectionsPositionIdentification(
      Pose &current_pose, Directions &nextDirectionsKnown,
      Directions &nextDirectionCandidates, bool &isForceGoingToGoal,
      int &matchCount);
  /**
   * @brief 迷路を編集してさらに優先順の精度を向上させる関数
   * @return 既知区間の最終区画
   */
  const Position calcNextDirectionsInAdvance(
      Maze &maze, const Positions &dest, const Pose &start_pose,
      Directions &nextDirectionsKnown, Directions &nextDirectionCandidates);
};

} // namespace MazeLib
