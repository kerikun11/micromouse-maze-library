/**
 * @file Agent.h
 * @brief 現在位置，探索状態を管理するクラスを定義するファイル
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
   * @brief 通常のコンストラクタ
   * @param maze 更新に用いる迷路への参照(書き込み権限あり)
   */
  Agent() : searchAlgorithm(maze) {}
  Agent(const Maze& maze) : maze(maze), searchAlgorithm(this->maze) {}
  /**
   * @brief 初期化
   */
  void reset() {
    maze.reset();
    next_directions.state = SearchAlgorithm::START;
    current_pose = Pose(Position(0, 1), Direction::North);
    isPositionIdentifying = false;
    isForceBackToStart = false;
    isForceGoingToGoal = false;
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
  void updateCurrentPose(const Pose& new_pose) {
    current_pose = new_pose;
    searchAlgorithm.updatePose(getState(), current_pose, isForceGoingToGoal);
  }
  /**
   * @brief 次に行くべき方向を取得する
   */
  bool determineNextDirection(const Pose& pose,
                              Direction& nextDirection) const {
    return searchAlgorithm.determineNextDirection(
        getState(), pose, next_directions.next_direction_candidates,
        nextDirection);
  }
  /**
   * @brief 壁を更新
   */
  bool updateWall(const Pose& pose,
                  const bool left,
                  const bool front,
                  const bool right) {
    return searchAlgorithm.updateWall(getState(), pose, left, front, right);
  }
  bool updateWall(const Position p, const Direction d, const bool b) {
    return searchAlgorithm.updateWall(getState(), p, d, b);
  }
  /**
   * @brief 壁を削除
   */
  void resetLastWalls(const int num = 1) {
    return searchAlgorithm.resetLastWalls(getState(), num);
  }
  /**
   * @brief 次に行くべき方向配列を計算
   * 注意: 処理に時間がかかる場合あり
   * @return 探索状態
   */
  SearchAlgorithm::Result calcNextDirections() {
    return searchAlgorithm.calcNextDirections(
        next_directions, current_pose, isPositionIdentifying,
        isForceBackToStart, isForceGoingToGoal);
  }
  /**
   * @brief 最短経路を導出
   * @param diag_enabled true: 斜めあり, false: 斜めなし
   * @return true: 成功, false: 失敗
   */
  bool calcShortestDirections(
      const bool diag_enabled,
      const StepMapSlalom::EdgeCost& edge_cost = StepMapSlalom::EdgeCost{}) {
    return searchAlgorithm.calcShortestDirections(shortest_dirs, diag_enabled,
                                                  edge_cost);
  }
  /**
   * @brief 探索を中止してスタート区画へ強制的に戻る
   * 時間が残りわずかな時などに使う
   */
  void setForceBackToStart(const bool yes = true) { isForceBackToStart = yes; }
  /**
   * @brief
   * たとえゴール区画が探索済みでも，一度ゴール区画を訪れるモードに設定する
   * 最短失敗後の自己位置同定後などに使用する
   */
  void setForceGoingToGoal(const bool yes = true) { isForceGoingToGoal = yes; }
  /**
   * @brief 自己位置同定モードに設定する
   */
  void setPositionIdentifying(const bool yes = true) {
    isPositionIdentifying = yes;
    if (yes) {
      searchAlgorithm.positionIdentifyingInit(current_pose);
      next_directions.state = SearchAlgorithm::IDENTIFYING_POSITION;
    } else {
      next_directions.state = SearchAlgorithm::START;
    }
  }
  /**
   * @brief 探索状態の取得
   */
  const SearchAlgorithm::State& getState() const {
    return next_directions.state;
  }
  /**
   * @brief 既知区間移動方向列を取得
   */
  const Directions& getNextDirectionsKnown() const {
    return next_directions.next_directions_known;
  }
  /**
   * @brief 壁を確認後に進む方向優先順位を取得
   */
  const Directions& getNextDirectionCandidates() const {
    return next_directions.next_direction_candidates;
  }
  /**
   * @brief 未知区間加速可能かどうかを取得
   */
  bool getUnknownAccelFlag() const {
    return next_directions.unknown_accel_flag;
  }
  /**
   * @brief 現在姿勢を取得
   */
  const Pose& getCurrentPose() const { return current_pose; }
  /**
   * @brief 最短経路の方向配列の計算結果を取得
   */
  const Directions& getShortestDirections() const { return shortest_dirs; }
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
   *
   * @param show_maze true:迷路も表示, false:迷路は非表示
   */
  void printInfo(const bool show_maze = true) const {
    printInfo(show_maze, current_pose, getState());
  }
  /**
   * @brief 探索状態の表示
   *
   * @param show_maze true:迷路も表示, false:迷路は非表示
   * @param pose ハイライトする区画姿勢
   * @param state 探索状態
   */
  void printInfo(const bool show_maze,
                 const Pose& pose,
                 const SearchAlgorithm::State state) const;
  /**
   * @brief 最短経路の表示
   */
  void printPath() const { maze.print(shortest_dirs, maze.getStart()); }
  /**
   * @brief 自己位置同定の候補数を取得
   */
  int getMatchCount() const { return next_directions.match_count; }

 protected:
  Maze maze;                          /**< @brief 探索に使用する迷路 */
  Pose current_pose;                  /**< @brief 現在の姿勢 */
  bool isForceBackToStart = false;    /**< @brief 強制帰還モード */
  bool isForceGoingToGoal = false;    /**< @brief 強制終点訪問モード */
  bool isPositionIdentifying = false; /**< @brief 自己位置同定モード */

 private:
  SearchAlgorithm searchAlgorithm; /**< @brief 探索器 */
  SearchAlgorithm::NextDirections
      next_directions;      /**< @brief 計算結果の移動方向配列 */
  Directions shortest_dirs; /**< @brief 最短経路の方向配列 */
};

}  // namespace MazeLib
