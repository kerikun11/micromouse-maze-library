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
    current_direction = Direction::North;
    current_position = Position(0, 0);
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
   * @brief 現在地を更新
   * @param p 区画座標
   * @param d 絶対方向
   */
  void updateCurrentPose(const Position p, const Direction d) {
    current_position = p;
    current_direction = d;
  }
  /**
   * @brief 次に行くべき方向を取得する
   */
  bool findNextDirection(const Position p, const Direction d,
                         Direction &nextDirection) const {
    return searchAlgorithm.findNextDirection(
        state, p, d, nextDirectionCandidates, nextDirection);
  }
  /**
   * @brief 絶対座標絶対方向で壁の1枚更新
   * @param p 区画座標
   * @param d 絶対方向
   * @param b 壁の有無
   */
  bool updateWall(const Position p, const Direction d, const bool left,
                  const bool front, const bool right, const bool back) {
    return searchAlgorithm.updateWall(state, p, d, left, front, right, back);
  }
  bool updateWall(const Position p, const Direction d, const bool b) {
    return searchAlgorithm.updateWall(state, p, d, b);
  }
  void resetLastWall(const int num = 1) {
    return searchAlgorithm.resetLastWall(state, num);
  }
  /**
   * @brief 次に行くべき方向配列を計算
   * 注意: 処理に時間がかかる場合あり
   * @return 探索状態
   */
  SearchAlgorithm::Result calcNextDirections() {
    return searchAlgorithm.calcNextDirections(
        state, current_position, current_direction, nextDirectionsKnown,
        nextDirectionCandidates, isPositionIdentifying, isForceBackToStart,
        isForceGoingToGoal, matchCount);
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
  void setForceBackToStart(bool yes = true) { isForceBackToStart = yes; }
  /**
   * @brief
   * たとえゴール区画が探索済みでも，一度ゴール区画を訪れるモードに設定する
   * 最短失敗後の自己位置同定後などに使用する
   */
  void setForceGoingToGoal(bool yes = true) { isForceGoingToGoal = yes; }
  /**
   * @brief 自己位置同定モードに設定する
   */
  void positionIdentify() {
    searchAlgorithm.positionIdentifyingInit(current_position,
                                            current_direction);
    state = SearchAlgorithm::IDENTIFYING_POSITION;
    isPositionIdentifying = true;
    calcNextDirections(); /*< 時間がかかる処理！ */
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
   * @brief 現在区画を取得
   */
  const Position &getCurrentPosition() const { return current_position; }
  /**
   * @brief 現在の方向を取得
   */
  const Direction &getCurrentDirection() const { return current_direction; }
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
   * @param showMaze true:迷路も表示, false:迷路は非表示
   */
  void printInfo(const bool showMaze = true) const {
    printInfo(showMaze, current_position, current_direction, state);
  }
  void printInfo(const bool showMaze, const Position vec, const Direction dir,
                 const SearchAlgorithm::State state) const;
  /**
   * @brief 最短経路の表示
   */
  void printPath() const { maze.print(shortest_dirs, maze.getStart()); }

protected:
  Maze &maze; /**< 使用する迷路の参照 */
  SearchAlgorithm::State state =
      SearchAlgorithm::START;         /**< 現在の探索状態を保持 */
  Position current_position;          /**< 現在の区画座標 */
  Direction current_direction;        /**< 現在向いている方向 */
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
