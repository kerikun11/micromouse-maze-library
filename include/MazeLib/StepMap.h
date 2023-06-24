/**
 * @file StepMap.h
 * @brief マイクロマウスの迷路の区画ベースのステップマップを扱うクラスを定義
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2017-11-05
 * @copyright Copyright 2017 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <limits> /*< for std::numeric_limits */

#include "MazeLib/Maze.h"

namespace MazeLib {

/**
 * @brief 区画ベースのステップマップを管理するクラス
 */
class StepMap {
 public:
  using step_t = uint16_t; /**< @brief ステップの型 */
  static constexpr step_t STEP_MAX =
      std::numeric_limits<step_t>::max(); /**< @brief 最大ステップ値 */

 public:
  /**
   * @brief デフォルトコンストラクタ
   * @details 台形加速のコストテーブルを計算する処理を含む
   */
  StepMap();
  /**
   * @brief ステップマップを初期化する関数
   * @param[in] step この値で全マップを初期化する
   */
  void reset(const step_t step = STEP_MAX) { stepMap.fill(step); }
  /**
   * @brief ステップの取得
   * @details 盤面外なら `STEP_MAX` を返す
   */
  step_t getStep(const int8_t x, const int8_t y) const {
    return getStep(Position(x, y));
  }
  /**
   * @brief ステップの取得
   * @details 盤面外なら `STEP_MAX` を返す
   */
  step_t getStep(const Position p) const {
    return p.isInsideOfField() ? stepMap[p.getIndex()] : STEP_MAX;
  }
  /**
   * @brief ステップの更新
   * @details 盤面外なら何もしない
   */
  void setStep(const int8_t x, const int8_t y, const step_t step) {
    return setStep(Position(x, y), step);
  }
  /**
   * @brief ステップの更新
   * @details 盤面外なら何もしない
   */
  void setStep(const Position p, const step_t step) {
    if (p.isInsideOfField()) stepMap[p.getIndex()] = step;
  }
  /**
   * @brief ステップマップの生配列への参照を取得 (読み取り専用)
   */
  const auto& getMapArray() const { return stepMap; }
  /**
   * @brief ステップのスケーリング係数を取得
   * @details ステップにこの数をかけるとミリ秒に変換できる
   */
  const auto getScalingFactor() const { return scalingFactor; }
  /**
   * @brief ステップの表示
   * @param[in] maze 表示する迷路
   * @param[in] p ハイライト区画
   * @param[in] d ハイライト方向
   * @param[inout] os output-stream
   */
  void print(const Maze& maze, const Position p = Position(-1, -1),
             const Direction d = Direction::Max,
             std::ostream& os = std::cout) const;
  void print(const Maze& maze, const Directions& dirs,
             const Position start = Position(0, 0),
             std::ostream& os = std::cout) const;
  void printFull(const Maze& maze, const Position p = Position(-1, -1),
                 const Direction d = Direction::Max,
                 std::ostream& os = std::cout) const;
  void printFull(const Maze& maze, const Directions& dirs,
                 const Position start = Position(0, 0),
                 std::ostream& os = std::cout) const;
  /**
   * @brief ステップマップの更新
   * @param[in] maze 更新に使用する迷路情報
   * @param[in] dest ステップを0とする目的地の区画の集合(順不同)
   * @param[in] knownOnly true:未知壁は通過不可能、false:未知壁は通過可能とする
   * @param[in] simple 台形加速を考慮せず、隣接区画のコストをすべて1にする
   */
  void update(const Maze& maze, const Positions& dest, const bool knownOnly,
              const bool simple);
  /**
   * @brief 与えられた区画間の最短経路を導出する関数
   * @param[in] maze 使用する迷路
   * @param[in] start 始点区画
   * @param[in] dest 目的地区画の集合(順不同)
   * @param[in] knownOnly 未知壁は壁ありとみなし、既知壁のみを使用する
   * @param[in] simple 台形加速を考慮せず、隣接区画のコストをすべて1にする
   * @return 始点区画から目的地区画への最短経路の方向列。
   *         経路がない場合は空配列となる。
   */
  Directions calcShortestDirections(const Maze& maze, const Position start,
                                    const Positions& dest, const bool knownOnly,
                                    const bool simple);
  /**
   * @brief スタートからゴールまでの最短経路を導出する関数
   * @param[in] maze 使用する迷路
   * @param[in] knownOnly 未知壁は壁ありとみなし、既知壁のみを使用する
   * @param[in] simple 台形加速を考慮せず、隣接区画のコストをすべて1にする
   * @return スタートからゴールへの最短経路の方向列。
   *         経路がない場合は空配列となる。
   */
  Directions calcShortestDirections(const Maze& maze, const bool knownOnly,
                                    const bool simple) {
    return calcShortestDirections(maze, maze.getStart(), maze.getGoals(),
                                  knownOnly, simple);
  }
  /**
   * @brief ステップマップから次に行くべき方向を計算する関数
   * @param[in] maze 使用する迷路
   * @param[in] start 移動開始位置
   * @param[out] nextDirectionsKnown 既知区間移動方向列
   * @param[out] nextDirectionCandidates 既知区間移動後の移動方向の優先順位
   * @return 既知区間の最終区画
   */
  Pose calcNextDirections(const Maze& maze, const Pose& start,
                          Directions& nextDirectionsKnown,
                          Directions& nextDirectionCandidates) const;
  /**
   * @brief ステップマップにより次に行くべき方向列を生成する
   * @param[in] maze 使用する迷路
   * @param[in] start 移動開始位置
   * @param[out] end 移動後の位置姿勢
   * @param[in] knownOnly 未知壁は壁ありとみなし、既知壁のみを使用する
   * @param[in] simple 台形加速を考慮せず、隣接区画のコストをすべて1にする
   * @param[in] breakUnknown 未知壁を含む区画に到達したら終了する(探索用)
   */
  Directions getStepDownDirections(const Maze& maze, const Pose& start,
                                   Pose& end, const bool knownOnly,
                                   const bool simple,
                                   const bool breakUnknown) const;
  /**
   * @brief 引数区画の周囲の未知壁の確認優先順位を生成する関数
   * @param[in] maze 使用する迷路
   * @param[in] focus 注目する区画の位置姿勢
   * @return 行くべき方向の優先順位
   */
  Directions getNextDirectionCandidates(const Maze& maze,
                                        const Pose& focus) const;
  /**
   * @brief ゴール区画内を行けるところまで直進させる方向列を追加する関数
   * @param[in] maze 使用する迷路
   * @param[inout] shortestDirections 追記元の方向列。これ自体に追記される。
   * @param[in] knownOnly 未知壁は壁ありとみなし、既知壁のみを使用する
   * @param[in] diagEnabled 斜めありなし
   */
  static void appendStraightDirections(const Maze& maze,
                                       Directions& shortestDirections,
                                       const bool knownOnly,
                                       const bool diagEnabled);

#if MAZE_DEBUG_PROFILING
  std::size_t queueSizeMax = 0;
#endif

 protected:
  /** @brief 迷路中のステップ数 */
  std::array<step_t, Position::SIZE> stepMap;
  /** @brief コストテーブルのサイズ */
  static constexpr int stepTableSize = MAZE_SIZE;
  /** @brief コストが最大値を超えないようにスケーリングする係数 */
  static constexpr float scalingFactor = 2;
  /** @brief 台形加速を考慮した移動コストテーブル (壁沿い方向) */
  std::array<step_t, MAZE_SIZE> stepTable;

  /**
   * @brief 計算の高速化のために予め直進のコストテーブルを計算する関数
   */
  void calcStraightCostTable();
};

}  // namespace MazeLib
