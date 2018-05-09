#pragma once

#include "Maze.h"
#include "SearchAlgorithm.h"
#include "PositionIdentifier.h"

namespace MazeLib {
  class Agent {
  public:
    Agent(Maze& maze, const Vectors& goal) : maze(maze), searchAlgorithm(maze, goal) { }
    void reset(){
      state = SearchAlgorithm::IDOLE;
      updateCurVecDir(Vector(0, 0), Dir::North);
    }
    void reset(const Vectors& goal) {
      searchAlgorithm.replaceGoal(goal);
      reset();
    }
    /** @function updateCurVecDir
    *   @brief 現在地を更新
    *   @param v 区画座標
    *   @param d 絶対方向
    */
    void updateCurVecDir(const Vector& v, const Dir& d){ curVec = v; curDir = d; }
    /** @function updateWall
    *   @brief 絶対座標絶対方向で壁の1枚更新
    *   @param v 区画座標
    *   @param d 絶対方向
    *   @param b 壁の有無
    */
    bool updateWall(const Vector& v, const Dir& d, const bool& b){
      if(isPositionIdentifying) return positionIdentifier.updateWall(v, d, b);
      // 既知の壁と食い違いがあったら未知壁とする
      if(maze.isKnown(v, d) && maze.isWall(v, d) != b){
        maze.setWall(v, d, false);
        maze.setKnown(v, d, false);
        return false;
      }
      if(!maze.isKnown(v, d)){
        maze.updateWall(v, d, b);
        wallLogs.push_back(WallLog(v, d, b));
      }
      return true;
    }
    bool updateWall(const Vector& v, const Dir& d, const bool left, const bool front, const bool right, Dir& nextDirInAdvance){
      updateWall(v, d+1, left); // left wall
      updateWall(v, d+0, front); // front wall
      updateWall(v, d-1, right); // right wall

      // 候補の中で行ける方向を探す
      nextDirInAdvance = *std::find_if(nextDirsInAdvance.begin(), nextDirsInAdvance.end(), [&](const Dir& dir){
        if(isPositionIdentifying) return positionIdentifier.getMaze().canGo(v, dir);
        return maze.canGo(v, dir);
      });
      return true;
    }
    bool resetLastWall(const int num = 1){
      for(int i=0;i<num;i++){
        if(wallLogs.empty()) return true;
        auto wl = wallLogs.back();
        maze.setWall(Vector(wl), wl.d, false);
        maze.setKnown(Vector(wl), wl.d, false);
        wallLogs.pop_back();
      }
    }
    /** @function calcNextDir
    *   @brief 次に行くべき方向配列を計算
    *   注意: 処理に時間がかかる場合あり
    *   @return 探索状態
    */
    bool calcNextDirs(){
      if(isPositionIdentifying) {
        Vector ans;
        auto res = positionIdentifier.identify(maze, ans);
        matchCount = res;
        if(res==1){
          curVec = ans + curVec - positionIdentifier.getStart();
          isPositionIdentifying = false;
        }else if(res==0) {
          return false;
        }else{
          return positionIdentifier.calcNextDirs(curVec, curDir, nextDirs, nextDirsInAdvance);
        }
      }
      return searchAlgorithm.calcNextDirs(state, curVec, curDir, nextDirs, nextDirsInAdvance, isForceBackToStart);
    }
    bool calcShortestDirs(const bool diagonal = true){
      return searchAlgorithm.calcShortestDirs(shortestDirs, diagonal);
    }
    /** @function forceBackToStart
    *   @brief 探索を中止してスタート区画へ強制的に戻る
    *   時間が残りわずかな時などに使う
    */
    void forceBackToStart(){
      isForceBackToStart = true;
    }
    void positionIdentify(const Dir d = Dir::North){
      curDir = d;
      curVec = positionIdentifier.getStart();
      isPositionIdentifying = true;
    }
    /** @function getState
    *   @brief 探索状態の取得
    */
    const SearchAlgorithm::State& getState() const {
      return state;
    }
    /** @function getNextDirs
    *   @brief 次に行くべき方向配列の計算結果を取得
    */
    const Dirs& getNextDirs() const {
      return nextDirs;
    }
    /** @function getNextDirs
    *   @brief 次に行くべき方向配列の計算結果を取得
    */
    const Dirs& getNextDirsInAdvance() const {
      return nextDirsInAdvance;
    }
    /** @function getCurVec
    *   @brief 現在区画を取得
    */
    const Vector& getCurVec() const {
      return curVec;
    }
    /** @function getCurDir
    *   @brief 現在の方向を取得
    */
    const Dir& getCurDir() const {
      return curDir;
    }
    /** @function getNextDirs
    *   @brief 最短経路の方向配列の計算結果を取得
    */
    const Dirs& getShortestDirs() const {
      return shortestDirs;
    }
    /** @function printInfo
    *   @brief 探索状態を表示
    *   @param showMaze true:迷路も表示, false:迷路は非表示
    */
    void printInfo(const bool& showMaze = true) const {
      // 迷路を表示
      if(showMaze) {
        // カーソルを移動
        for(int i=0; i<10; i++) printf("\x1b[A");
        if(isPositionIdentifying){
          positionIdentifier.printMap(curVec, curDir);
        }else{
          searchAlgorithm.printMap(state, curVec, curDir);
        }
      }
      // 詳細を表示
      printf("Cur: ( %2d, %2d,  %c), State: %s       \n", curVec.x, curVec.y, ">^<v"[curDir], SearchAlgorithm::stateString(state));
      printf("nextDirs: ");
      for (const auto d : getNextDirs()) printf("%c", ">^<v"[d]);
      printf("                                                     \n");
      printf("nextDirsInAdvance: ");
      for(const auto d: getNextDirsInAdvance()) printf("%c", ">^<v"[d]);
      printf("        \n");
      printf("Match Count: %d      \n", matchCount);
    }
    /** @function printPath
    *   @brief 最短経路の表示
    */
    void printPath() const {
      maze.printPath(Vector(0, 0), shortestDirs);
      printf("Shortest Step: %d\n", shortestDirs.size());
    }
    WallLogs& getWallLog(){
      return wallLogs;
    }

  private:
    Maze& maze; /**< 使用する迷路の参照 */
    SearchAlgorithm searchAlgorithm; /**< 探索器 */
    PositionIdentifier positionIdentifier; /**< 自己位置推定器 */
    SearchAlgorithm::State state; /**< 現在の探索状態を保持 */
    Vector curVec; /**< 現在の区画座標 */
    Dir curDir; /**< 現在向いている方向 */
    WallLogs wallLogs;
    Dirs nextDirs; /**< 次に行くべき探索方向配列 */
    Dirs nextDirsInAdvance; /**< 最短経路の方向配列 */
    Dirs shortestDirs; /**< 最短経路の方向配列 */
    bool isForceBackToStart = false;
    bool isPositionIdentifying = false;
    int matchCount = 0;
  };
}
