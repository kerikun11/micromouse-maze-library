/**
 * @file Agent.h
 * @brief 現在位置、探索状態を管理するクラスを定義するファイル
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-05-20
 * @copyright Copyright 2018 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include "MazeLib/Maze.h"
#include "MazeLib/SearchAlgorithm.h"

namespace MazeLib {

/**
 * @brief 自己位置や探索状態などを保持するクラス
 */
class Agent {
 public:
  /**
   * @brief 通常のコンストラクタ。未探索状態から開始。
   */
  Agent() : searchAlgorithm(maze) {}
  /**
   * @brief 探索情報を消去して初期化する
   */
  void reset() {
    maze.reset();
    calcData = SearchAlgorithm::CalcData();
    calcData.currentPose = Pose(Position(0, 1), Direction::North);
  }
  /**
   * @brief ゴール区画を変更する関数
   */
  void replaceGoals(const Positions& goals) { maze.setGoals(goals); }
  /**
   * @brief 探索が完了しているかどうかを返す関数
   */
  bool isCompleted() { return searchAlgorithm.isCompleted(); }
  /**
   * @brief ゴールが封印されていないか確認する関数
   */
  bool isSolvable() { return searchAlgorithm.isSolvable(); }
  /**
   * @brief 現在地を更新
   */
  void updateCurrentPose(const Pose& newPose) {
    return searchAlgorithm.updatePose(calcData, newPose);
  }
  /**
   * @brief 次に行くべき方向を取得する
   * @param[out] nextDirection 次に行くべき方向を格納する
   */
  bool determineNextDirection(Direction& nextDirection) const {
    return searchAlgorithm.determineNextDirection(calcData, nextDirection);
  }
  /**
   * @brief 壁情報を更新する
   * @param left 現在姿勢から見た右方向の壁の有無
   * @param front 現在姿勢から見た前方向の壁の有無
   * @param right 現在姿勢から見た左方向の壁の有無
   * @retval true 正常に壁情報を更新できた
   * @retval false 壁情報に食い違いがあった
   */
  bool updateWall(const bool left, const bool front, const bool right) {
    return searchAlgorithm.updateWall(getState(), getCurrentPose(), left, front,
                                      right);
  }
  bool updateWall(const Position p, const Direction d, const bool b) {
    return searchAlgorithm.updateWall(getState(), p, d, b);
  }
  /**
   * @brief 直近で見た壁情報を削除
   */
  void resetLastWalls(const int num = 1) {
    return searchAlgorithm.resetLastWalls(getState(), num);
  }
  /**
   * @brief 次に行くべき方向配列を計算
   * @attention この処理には時間がかかる
   * @return 計算結果
   */
  SearchAlgorithm::Result calcNextDirections() {
    return searchAlgorithm.calcNextDirections(calcData);
  }
  /**
   * @brief 最短経路を導出
   * @param diagEnabled true: 斜めあり, false: 斜めなし
   * @param edgeCost StepMapSlalom の走行パラメータ
   * @return true: 成功, false: 失敗
   */
  bool calcShortestDirections(
      const bool diagEnabled,
      const StepMapSlalom::EdgeCost& edgeCost = StepMapSlalom::EdgeCost()) {
    return searchAlgorithm.calcShortestDirections(shortestDirections,
                                                  diagEnabled, edgeCost);
  }
  /**
   * @brief 探索を中止してスタート区画へ強制的に戻る
   * @details 時間が残りわずかな時などに使う
   * @attention ゴール区画へのルートが未探索の場合は探索を続行する
   */
  void setForceBackToStart(const bool yes = true) {
    calcData.isForceBackToStart = yes;
  }
  /**
   * @brief ゴール区画訪問モードに設定する
   * @details 最短失敗後の自己位置同定後などに使用する
   */
  void setForceGoingToGoal(const bool yes = true) {
    calcData.isForceGoingToGoal = yes;
  }
  /**
   * @brief 自己位置同定モードに設定する
   */
  void setPositionIdentifying(const bool yes = true) {
    calcData.isPositionIdentification = yes;
    if (yes) {
      searchAlgorithm.positionIdentifyingInit(calcData.currentPose);
      calcData.result.state = SearchAlgorithm::IDENTIFYING_POSITION;
    } else {
      calcData.result.state = SearchAlgorithm::START;
    }
  }
  /**
   * @brief 探索状態の取得
   */
  const SearchAlgorithm::State& getState() const {
    return calcData.result.state;
  }
  /**
   * @brief 既知区間移動方向列を取得
   */
  const Directions& getNextDirectionsKnown() const {
    return calcData.result.nextDirectionsKnown;
  }
  /**
   * @brief 壁を確認後に進む方向の優先順位を取得
   */
  const Directions& getNextDirectionCandidates() const {
    return calcData.result.nextDirectionCandidates;
  }
  /**
   * @brief 未知区間加速可能かどうかを取得
   */
  bool getUnknownAccelFlag() const { return calcData.result.unknownAccelFlag; }
  /**
   * @brief 現在姿勢を取得
   */
  const Pose& getCurrentPose() const { return calcData.currentPose; }
  /**
   * @brief 最短経路の方向配列の計算結果を取得
   */
  const Directions& getShortestDirections() const { return shortestDirections; }
  /**
   * @brief 迷路を取得
   */
  const Maze& getMaze() const { return maze; }
  /**
   * @brief 迷路を更新
   */
  void updateMaze(const Maze& new_maze) { maze = new_maze; }
  /**
   * @brief 探索器を取得
   */
  const SearchAlgorithm& getSearchAlgorithm() const { return searchAlgorithm; }
  /**
   * @brief 探索状態の表示
   * @param showMaze true:迷路も表示 false:迷路は非表示
   */
  void printInfo(const bool showMaze = false) const;
  /**
   * @brief 最短経路の表示
   */
  void printPath() const { maze.print(shortestDirections, maze.getStart()); }
  /**
   * @brief 自己位置同定の候補数を取得
   */
  int getMatchCount() const { return calcData.result.positionIdMatchedCount; }

 protected:
  Maze maze;                          /**< @brief 探索に使用する迷路 */
  SearchAlgorithm searchAlgorithm;    /**< @brief 探索器 */
  SearchAlgorithm::CalcData calcData; /**< @brief 計算結果 */
  Directions shortestDirections;      /**< @brief 最短経路の方向配列 */
};

}  // namespace MazeLib
