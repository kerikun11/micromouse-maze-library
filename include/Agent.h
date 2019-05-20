/**
 *  @file Agent.h
 *  @brief マイクロマウスの探索ナビゲータ
 *  @author KERI (Github: kerikun11)
 *  @url https://kerikeri.top/
 *  @date 2018.05.20
 */
#pragma once

#include "Maze.h"
#include "SearchAlgorithm.h"

namespace MazeLib {
class Agent {
public:
  Agent(Maze &maze) : maze(maze), searchAlgorithm(maze) {}
  /** @function replaceGoals
   *  @brief ゴール区画を変更する関数
   */
  void replaceGoals(const Vectors &goals) { maze.setGoals(goals); }
  /** @function isComplete
   *  @brief 探索が完了しているかどうかを返す関数
   */
  bool isComplete() { return searchAlgorithm.isComplete(); }
  /** @function updateCurVecDir
   *  @brief 現在地を更新
   *  @param v 区画座標
   *  @param d 絶対方向
   */
  void updateCurVecDir(const Vector &v, const Dir &d) {
    curVec = v;
    curDir = d;
  }
  /** @function findNextDir
   *  @brief 次に行くべき方向を取得する
   */
  bool findNextDir(const Vector v, const Dir d, Dir &nextDir) const {
    return searchAlgorithm.findNextDir(state, v, d, nextDirCandidates, nextDir);
  }
  /** @function updateWall
   *  @brief 絶対座標絶対方向で壁の1枚更新
   *  @param v 区画座標
   *  @param d 絶対方向
   *  @param b 壁の有無
   */
  bool updateWall(const Vector &v, const Dir &d, const bool left,
                  const bool front, const bool right, const bool back) {
    return searchAlgorithm.updateWall(state, v, d, left, front, right, back);
  }
  void resetLastWall(const int num = 1) {
    return searchAlgorithm.resetLastWall(state, num);
  }
  /** @function calcNextDirs
   *  @brief 次に行くべき方向配列を計算
   *  注意: 処理に時間がかかる場合あり
   *  @return 探索状態
   */
  SearchAlgorithm::Status calcNextDirs() {
    return searchAlgorithm.calcNextDirs(
        state, curVec, curDir, nextDirsKnown, nextDirCandidates,
        isPositionIdentifying, isForceBackToStart, isForceGoingToGoal,
        matchCount);
  }
  /** @function calcShortestDirs
   *  @brief 最短経路を導出
   *  @param diagonal true: 斜めあり, false: 斜めなし
   *  @return true: 成功, false: 失敗
   */
  bool calcShortestDirs(const bool diagonal = true) {
    return searchAlgorithm.calcShortestDirs(shortestDirs, diagonal);
  }
  /** @function forceBackToStart
   *  @brief 探索を中止してスタート区画へ強制的に戻る
   *  時間が残りわずかな時などに使う
   */
  void forceBackToStart(bool yes = true) { isForceBackToStart = yes; }
  /**
   * @brief
   * たとえゴール区画が探索済みでも，一度ゴール区画を訪れるモードに設定する
   * 最短失敗後の自己位置同定後などに使用する
   */
  void forceGoingToGoal(bool yes = true) { isForceGoingToGoal = yes; }
  /**
   * @brief 自己位置同定モードに設定する
   */
  void positionIdentify() {
    searchAlgorithm.positionIdentifyingInit(&curVec, &curDir);
    state = SearchAlgorithm::IDENTIFYING_POSITION;
    isPositionIdentifying = true;
  }
  /** @function getState
   *  @brief 探索状態の取得
   */
  const SearchAlgorithm::State &getState() const { return state; }
  /** @function getNextDirs
   *  @brief 次に行くべき方向配列の計算結果を取得
   */
  const Dirs &getNextDirs() const { return nextDirsKnown; }
  /** @function getNextDirs
   *  @brief 次に行くべき方向配列の計算結果を取得
   */
  const Dirs &getNextDirCandidates() const { return nextDirCandidates; }
  /** @function getCurVec
   *  @brief 現在区画を取得
   */
  const Vector &getCurVec() const { return curVec; }
  /** @function getCurDir
   *  @brief 現在の方向を取得
   */
  const Dir &getCurDir() const { return curDir; }
  /** @function getNextDirs
   *  @brief 最短経路の方向配列の計算結果を取得
   */
  const Dirs &getShortestDirs() const { return shortestDirs; }
  /** @function getMaze
   *  @brief 迷路を取得
   */
  Maze &getMaze() { return maze; }
  /** @function printInfo
   *  @brief 探索状態を表示
   *  @param showMaze true:迷路も表示, false:迷路は非表示
   */
  void printInfo(const bool showMaze = true) const {
    printInfo(showMaze, curVec, curDir, state);
  }
  void printInfo(const bool showMaze, const Vector vec, const Dir dir,
                 const SearchAlgorithm::State state) const;
  /** @function printPath
   *  @brief 最短経路の表示
   */
  void printPath() const {
    maze.printPath(maze.getStart(), shortestDirs);
    std::cout << "Shortest Step: " << shortestDirs.size() << std::endl;
  }

protected:
  Maze &maze; /**< 使用する迷路の参照 */
  SearchAlgorithm::State state =
      SearchAlgorithm::START;         /**< 現在の探索状態を保持 */
  Vector curVec;                      /**< 現在の区画座標 */
  Dir curDir;                         /**< 現在向いている方向 */
  bool isForceBackToStart = false;    /**< 強制帰還モード */
  bool isForceGoingToGoal = false;    /**< 強制終点訪問モード */
  bool isPositionIdentifying = false; /**< 自己位置同定モード */

private:
  SearchAlgorithm searchAlgorithm; /**< 探索器 */
  Dirs nextDirsKnown;              /**< 次に行く探索方向配列 */
  Dirs nextDirCandidates; /**< 次に行く方向の候補の優先順 */
  Dirs shortestDirs;      /**< 最短経路の方向配列 */
  int matchCount = 0;     /**< 自己位置同定の候補数，表示用 */
};
} // namespace MazeLib
