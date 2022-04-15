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
   * @brief 探索状態
   */
  enum State : uint8_t {
    START,                  /**< @brief 初期位置、初期姿勢 */
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
   * @brief 移動方向の計算結果の構造体
   */
  struct CalcResultData {
    State state = State::START;     /**< @brief 探索状態 */
    Directions nextDirectionsKnown; /**< @brief 既地区間移動候補列 */
    Directions nextDirectionCandidates; /**< @brief 未知区間移動候補順位 */
    bool unknownAccelFlag = false; /**< @brief 未知区間加速可能フラグ */
    int positionIdMatchedCount = 0; /**< @brief 自己位置同定の候補数 */
  };
  /**
   * @brief 移動方向の計算の入出力構造体
   */
  struct CalcData {
    Pose currentPose;                      /**< @brief 現在の姿勢 */
    bool isPositionIdentification = false; /**< @brief 自己位置同定モード */
    bool isForceBackToStart = false;       /**< @brief 強制帰還モード */
    bool isForceGoingToGoal = false; /**< @brief 強制終点訪問モード */
    CalcResultData result; /**< @brief 移動方向列の計算結果 */
  };

 public:
  /**
   * @brief デフォルトコンストラクタ
   * @param maze 使用する迷路への参照。コピーせずに参照として引き継ぐ。
   */
  SearchAlgorithm(Maze& maze) : maze(maze) {}
  /**
   * @brief 探索が完全に終了しているかどうかを返す関数
   * @attention 計算に時間がかかる可能性あり
   */
  bool isCompleted();
  /**
   * @brief 探索によって可解かどうかを返す関数
   * @attention 計算に時間がかかる可能性あり
   */
  bool isSolvable();

  /**
   * @brief 壁の更新
   * @details 現在の探索状態を考慮して迷路情報を更新する
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
   * @details calcData のゴール区画の訪問フラグなども更新する
   * @param[inout] calcData 更新先のデータ
   * @param[in] newPose 更新元の姿勢
   */
  void updatePose(CalcData& calcData, const Pose& newPose);

  /**
   * @brief 次に進むべき方向を計算する関数
   * @param[inout] calcData 計算条件と計算結果の構造体
   * @return Result 計算結果
   */
  Result calcNextDirections(CalcData& calcData);
  /**
   * @brief 次に進むべき候補列から次に進むべき方向を決定する関数
   * @param[in] calcData 事前計算の結果
   * @param[out] nextDirection 次に進むべき方向
   * @retval true 成功
   * @retval false 失敗
   */
  bool determineNextDirection(const CalcData& calcData,
                              Direction& nextDirection) const;
  /**
   * @brief 最短経路の導出
   * @param[out] shortestDirections 最短経路を格納する方向列
   * @param[in] diagEnabled 斜め走行を有効化する
   * @param[in] edgeCost 加速・スラロームの重み
   * @retval true 成功
   * @retval false 失敗
   */
  bool calcShortestDirections(
      Directions& shortestDirections,
      const bool diagEnabled = true,
      const StepMapSlalom::EdgeCost& edgeCost = StepMapSlalom::EdgeCost());
  /**
   * @brief 自己位置同定の初期化
   * @param[out] currentPose 初期姿勢
   */
  void positionIdentifyingInit(Pose& currentPose);
  /**
   * @brief ステップマップの表示
   */
  void printStepMap(const State state, const Pose& pose) const;

  /* Getters */
  const Maze& getMaze() const { return maze; }
  const StepMap& getStepMap() const { return stepMap; }
  const StepMapWall& getStepMapWall() const { return stepMapWall; }
  const StepMapSlalom& getStepMapSlalom() const { return stepMapSlalom; }
  const Maze& getIdMaze() const { return idMaze; }
  const uint32_t& getShortestCost() const { return shortestCost; }

 protected:
  Maze& maze;                  /**< @brief 使用する迷路の参照 */
  StepMap stepMap;             /**< @brief 使用するステップマップ */
  StepMapWall stepMapWall;     /**< @brief 使用するステップマップ */
  StepMapSlalom stepMapSlalom; /**< @brief 使用するステップマップ */
  uint32_t shortestCost;       /**< @brief 最短経路のコスト [ms] */

 protected:
  Maze idMaze;       /**< @brief 自己位置同定に使用する迷路 */
  Position idOffset; /**< @brief 自己位置同定迷路の始点位置 */

  /**
   * @brief ステップマップにより最短経路上になりうる区画を洗い出す
   * @details[out] candidates 最短経路になりうる未探索区画
   * @details[in] currentPose 現在姿勢
   */
  bool findShortestCandidates(Positions& candidates,
                              const Pose& currentPose = Pose());
  /**
   * @brief 自己位置同定のパターンにマッチする候補をカウントする
   * @param[in] idWallRecords 自己位置同定走行の壁ログ
   * @param[out] matchedPose マッチした姿勢(のひとつ)
   * @return int 自己位置のマッチ数, 0: 失敗, 1: 特定, 2-: 複数マッチ (同定中)
   * @retval 0 失敗
   * @retval 1 特定
   * @retval 2- 複数マッチ (同定中)
   */
  int countIdentityCandidates(const WallRecords& idWallRecords,
                              Pose& matchedPose) const;
  /**
   * @brief 特定の区画にマッチする方向の候補を探す
   * @details スタート区画への訪問を避けるために使用する関数
   * @param[in] currentPosition 注目する区画 (自己位置同定用迷路上の座標)
   * @param[in] targetPose 検索対象の区画と方向 (探索用迷路上の座標)
   * @param[in] ignoreWalls 食い違いを許容する壁の数
   * @return const Directions 注目する区画からの方向列
   */
  const Directions findMatchDirectionCandidates(const Position currentPosition,
                                                const Pose& targetPose,
                                                const int ignoreWalls) const;

  /**
   * @brief 各状態での進行方向列導出関数
   * @param[inout] calcData 計算条件と結果の構造体
   * @return 計算結果
   */
  Result calcNextDirectionsSearchForGoal(CalcData& calcData);
  Result calcNextDirectionsSearchAdditionally(CalcData& calcData);
  Result calcNextDirectionsBackingToStart(CalcData& calcData);
  Result calcNextDirectionsGoingToGoal(CalcData& calcData);
  Result calcNextDirectionsPositionIdentification(CalcData& calcData);
  /**
   * @brief 迷路を編集してさらに優先順の精度を向上させる関数
   * @param[in] maze 使用する迷路
   * @param[in] dest ゴール区画の集合
   * @param[inout] calcData 事前計算の結果
   * @return Pose 既知区間の最終姿勢
   */
  const Pose calcNextDirectionsInAdvance(Maze& maze,
                                         const Positions& dest,
                                         CalcData& calcData);
};

}  // namespace MazeLib
