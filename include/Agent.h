/**
 * @file Agent.h
 * @brief 現在位置，探索状態を管理するクラスを定義するファイル
 * @author KERI (Github: kerikun11)
 * @url https://kerikeri.top/
 * @date 2018.05.20
 */
#pragma once

#include "Maze.h"
#include "SearchAlgorithm.h"

namespace MazeLib {

/**
 * @brief 自己位置を管理するクラス
 */
class Agent {
public:
  Agent(Maze &maze) : maze(maze), searchAlgorithm(maze) {}
  /**
   * @brief 初期化
   */
  void reset() {
    maze.reset();
    state = SearchAlgorithm::START;
    current_pose = Pose(Position(0, 1), Direction::North);
    isPositionIdentifying = false;
    isForceBackToStart = false;
    isForceGoingToGoal = false;
  }
  /**
   * @brief ゴール区画を変更する関数
   */
  void replaceGoals(const Positions &goals) { maze.setGoals(goals); }
  /**
   * @brief 探索が完了しているかどうかを返す関数
   */
  bool isComplete() { return searchAlgorithm.isComplete(); }
  /**
   * @brief ゴールが封印されていないか確認する関数
   */
  bool isSolvable() { return searchAlgorithm.isSolvable(); }
  /**
   * @brief 現在地を更新
   * @param p 区画座標
   * @param d 絶対方向
   */
  void updateCurrentPose(const Pose &new_pose) {
    current_pose = new_pose;
    searchAlgorithm.updatePose(state, current_pose, isForceGoingToGoal);
  }
  /**
   * @brief 次に行くべき方向を取得する
   */
  bool determineNextDirection(const Pose &pose,
                              Direction &nextDirection) const {
    return searchAlgorithm.determineNextDirection(
        state, pose, nextDirectionCandidates, nextDirection);
  }
  /**
   * @brief 壁を更新
   */
  bool updateWall(const Pose &pose, const bool left, const bool front,
                  const bool right) {
    return searchAlgorithm.updateWall(state, pose, left, front, right);
  }
  bool updateWall(const Position &p, const Direction d, const bool b) {
    return searchAlgorithm.updateWall(state, p, d, b);
  }
  /**
   * @brief 壁を削除
   */
  void resetLastWalls(const int num = 1) {
    return searchAlgorithm.resetLastWalls(state, num);
  }
  /**
   * @brief 次に行くべき方向配列を計算
   * 注意: 処理に時間がかかる場合あり
   * @return 探索状態
   */
  SearchAlgorithm::Result calcNextDirections() {
    return searchAlgorithm.calcNextDirections(
        state, current_pose, nextDirectionsKnown, nextDirectionCandidates,
        isPositionIdentifying, isForceBackToStart, isForceGoingToGoal,
        matchCount);
  }
  /**
   * @brief 最短経路を導出
   * @param diagonal true: 斜めあり, false: 斜めなし
   * @return true: 成功, false: 失敗
   */
  bool calcShortestDirections(const bool diag_enabled) {
    return searchAlgorithm.calcShortestDirections(shortest_dirs, diag_enabled);
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
      state = SearchAlgorithm::IDENTIFYING_POSITION;
      calcNextDirections(); /*< 時間がかかる処理！ */
    } else {
      state = SearchAlgorithm::START;
    }
  }
  /**
   * @brief 探索状態の取得
   */
  const SearchAlgorithm::State &getState() const { return state; }
  /**
   * @brief 次に行くべき方向配列の計算結果を取得
   */
  const Directions &getNextDirections() const { return nextDirectionsKnown; }
  /**
   * @brief 次に行くべき方向配列の計算結果を取得
   */
  const Directions &getNextDirectionCandidates() const {
    return nextDirectionCandidates;
  }
  /**
   * @brief 現在姿勢を取得
   */
  const Pose &getCurrentPose() const { return current_pose; }
  /**
   * @brief 最短経路の方向配列の計算結果を取得
   */
  const Directions &getShortestDirections() const { return shortest_dirs; }
  /**
   * @brief 迷路を取得
   */
  const Maze &getMaze() const { return maze; }
  /**
   * @brief Get the Search Algorithm object
   */
  const SearchAlgorithm &getSearchAlgorithm() const { return searchAlgorithm; }
  /**
   * @brief 探索状態を表示
   * @param show_maze true:迷路も表示, false:迷路は非表示
   */
  void printInfo(const bool show_maze = true) const {
    printInfo(show_maze, current_pose, state);
  }
  void printInfo(const bool show_maze, const Pose &pose,
                 const SearchAlgorithm::State state) const;
  /**
   * @brief 最短経路の表示
   */
  void printPath() const { maze.print(shortest_dirs, maze.getStart()); }
  /**
   * @brief Get the Match Count Value
   */
  int getMatchCount() const { return matchCount; }

protected:
  Maze &maze; /**< 使用する迷路の参照 */
  SearchAlgorithm::State state =
      SearchAlgorithm::START;         /**< 現在の探索状態を保持 */
  Pose current_pose;                  /**< 現在の姿勢 */
  bool isForceBackToStart = false;    /**< 強制帰還モード */
  bool isForceGoingToGoal = false;    /**< 強制終点訪問モード */
  bool isPositionIdentifying = false; /**< 自己位置同定モード */

private:
  SearchAlgorithm searchAlgorithm; /**< 探索器 */
  Directions nextDirectionsKnown;  /**< 次に行く既知方向配列 */
  Directions nextDirectionCandidates; /**< 次に行く未知方向候補の優先順 */
  Directions shortest_dirs;           /**< 最短経路の方向配列 */
  int matchCount = 0; /**< 自己位置同定の候補数，表示用 */
};

} // namespace MazeLib
