/**
 * @file SearchAlgorithm.h
 * @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2017-11-05
 * @copyright Copyright 2017 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include "MazeLib/Maze.h"
#include "MazeLib/StepMap.h"
#include "MazeLib/StepMapSlalom.h"
#include "MazeLib/StepMapWall.h"

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
    Processing, /**< @brief 現探索状態を継続 */
    Reached,    /**< @brief 現探索状態が完了 */
    Error,      /**< @brief エラーが発生 */
  };
  /**
   * @brief 探索状態を列挙
   */
  enum State : uint8_t {
    START,                  /**< @brief 初期位置，初期姿勢 */
    SEARCHING_FOR_GOAL,     /**< @brief ゴール区画を探索中 */
    SEARCHING_ADDITIONALLY, /**< @brief 追加探索中 */
    BACKING_TO_START,       /**< @brief スタートに戻っている */
    REACHED_START,          /**< @brief スタートに戻ってきた */
    IMPOSSIBLE,             /**< @brief ゴールは到達不可能な場所 */
    IDENTIFYING_POSITION,   /**< @brief 自己位置同定中 */
    GOING_TO_GOAL,          /**< @brief ゴールへ向かっている */
  };
  /**
   * @brief Stateの表示用文字列を返す関数
   */
  static const char* getStateString(const State s);
  /**
   * @brief 計算結果の構造体
   */
  struct NextDirections {
    State state = State::START;     /**< @brief 探索状態 */
    Directions nextDirectionsKnown; /**< @brief 既地区間移動候補列 */
    Directions nextDirectionCandidates; /**< @brief 未知区間移動候補順位 */
    bool unknownAccelFlag = false; /**< @brief 未知区間加速可能フラグ */
    int poseMatchCount = 0;        /**< @brief 自己位置同定の候補数 */
  };

 public:
  /**
   * @brief デフォルトコンストラクタ
   * @param maze 使用する迷路への参照
   */
  SearchAlgorithm(Maze& maze) : maze(maze) {}
  /**
   * @brief 探索が完全に終了しているかどうかを返す関数
   */
  bool isCompleted();
  /**
   * @brief 探索によって可解かどうかを返す関数
   */
  bool isSolvable();

  /**
   * @brief 壁の更新
   */
  bool updateWall(const State state,
                  const Pose& pose,
                  const bool left,
                  const bool front,
                  const bool right);
  bool updateWall(const State state,
                  const Position p,
                  const Direction d,
                  const bool b);
  /**
   * @brief 直近の壁情報の削除
   */
  void resetLastWalls(const State state, const int num = 1);
  /**
   * @brief 探索状態に応じた現在姿勢の更新
   */
  void updatePose(const State& state,
                  Pose& currentPose,
                  bool& isForceGoingToGoal);

  /**
   * @brief 次に進むべき方向を計算する関数
   *
   * @param[out] nextDirections 計算結果を格納する構造体
   * @param[inout] currentPose 現在位置（自己位置同定の場合は変更されうる）
   * @param[inout] isPositionIdentifying 自己位置同定フラグ
   * @param[inout] isForceBackToStart 強制帰還フラグ
   * @param[inout] isForceGoingToGoal 強制ゴール訪問フラグ
   * @return Result 計算結果
   */
  Result calcNextDirections(NextDirections& nextDirections,
                            Pose& currentPose,
                            bool& isPositionIdentifying,
                            bool& isForceBackToStart,
                            bool& isForceGoingToGoal);
  /**
   * @brief 次に進むべき候補列から次に進むべき方向を決定する関数
   *
   * @param[in] pose 現在姿勢
   * @param[in] nextDirectionCandidates
   * @param[out] nextDirection 次に進むべき方向
   * @return true 成功
   * @return false 失敗
   */
  bool determineNextDirection(const Maze& maze,
                              const Pose& pose,
                              const Directions& nextDirectionCandidates,
                              Direction& nextDirection) const;
  /**
   * @brief 次に進むべき候補列から次に進むべき方向を決定する関数
   *
   * @param[in] state 現在状態
   * @param[in] pose 現在姿勢
   * @param[in] nextDirectionCandidates
   * @param[out] nextDirection 次に進むべき方向
   * @return true 成功
   * @return false 失敗
   */
  bool determineNextDirection(const State state,
                              const Pose& pose,
                              const Directions& nextDirectionCandidates,
                              Direction& nextDirection) const;

  /**
   * @brief 最短経路の導出
   *
   * @param[out] shortestDirections 最短経路を格納する方向列
   * @param[in] diagEnabled オプション
   * @param[in] edgeCost 加速・スラロームの重み
   * @return true 成功
   * @return false 失敗
   */
  bool calcShortestDirections(
      Directions& shortestDirections,
      const bool diagEnabled = true,
      const StepMapSlalom::EdgeCost& edgeCost = StepMapSlalom::EdgeCost());

  /**
   * @brief 自己位置同定の初期化
   *
   * @param[out] currentPose 初期姿勢
   */
  void positionIdentifyingInit(Pose& currentPose);

  /**
   * @brief 表示
   */
  void printMap(const State state, const Pose& pose) const;

  /**
   * @brief Getters
   */
  const Maze& getMaze() const { return maze; }
  const StepMap& getStepMap() const { return step_map; }
  const StepMapWall& getStepMapWall() const { return step_map_wall; }
  const StepMapSlalom& getStepMapSlalom() const { return step_map_slalom; }
  const Maze& getIdMaze() const { return idMaze; }
  const uint32_t& getShortestCost() const { return cost; }

 protected:
  Maze& maze;                /**< @brief 使用する迷路の参照 */
  StepMap step_map;          /**< @brief 使用するステップマップ */
  StepMapWall step_map_wall; /**< @brief 使用するステップマップ */
  StepMapSlalom step_map_slalom; /**< @brief 使用するステップマップ */
  uint32_t cost;                 /*< 最短経路のコストを保持 */

 protected:
  Maze idMaze;       /**< @brief 自己位置同定に使用する迷路 */
  Position idOffset; /**< @brief 自己位置同定迷路の始点位置 */

  /**
   * @brief ステップマップにより最短経路上になりうる区画を洗い出す
   */
  bool findShortestCandidates(Positions& candidates,
                              const Pose& currentPose = Pose());
  /**
   * @brief 自己位置同定のパターンにマッチする候補をカウントする
   *
   * @param idWallRecords[in] 自己位置同定走行の壁ログ
   * @param ans[out] マッチした解のひとつ
   * @return int マッチ数, 0: 失敗, 1: 特定, 2-: 複数マッチ (同定中)
   * @retval 0 失敗
   * @retval 1 特定
   * @retval 2- 複数マッチ (同定中)
   */
  int countIdentityCandidates(const WallRecords& idWallRecords,
                              Pose& ans) const;
  /**
   * @brief 特定の区画にマッチする方向の候補を探す
   * スタート区画への訪問を避けるために使用する関数
   *
   * @param currentPosition 注目する区画
   * @param targetPose 検索対象の区画と方向
   * @return const Directions 注目する区画からの方向列
   */
  const Directions findMatchDirectionCandidates(const Position currentPosition,
                                                const Pose& targetPose) const;

  /**
   * @brief 各状態での進行方向列導出関数
   */
  Result calcNextDirectionsSearchForGoal(NextDirections& nextDirections,
                                         const Pose& currentPose);
  Result calcNextDirectionsSearchAdditionally(NextDirections& nextDirections,
                                              const Pose& currentPose);
  Result calcNextDirectionsBackingToStart(NextDirections& nextDirections,
                                          const Pose& currentPose);
  Result calcNextDirectionsGoingToGoal(NextDirections& nextDirections,
                                       const Pose& currentPose);
  Result calcNextDirectionsPositionIdentification(
      NextDirections& nextDirections,
      Pose& currentPose,
      bool& isForceGoingToGoal);
  /**
   * @brief 迷路を編集してさらに優先順の精度を向上させる関数
   * @return 既知区間の最終区画
   */
  const Position calcNextDirectionsInAdvance(Maze& maze,
                                             const Positions& dest,
                                             const Pose& startPose,
                                             NextDirections& nextDirections);
};

}  // namespace MazeLib
