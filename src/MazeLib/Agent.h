#pragma once

#include "Maze.h"
#include "SearchAlgorithm.h"

namespace MazeLib {
  class Agent {
  public:
    Agent(const Vectors& goal) : searchAlgorithm(maze, goal) {}
    /** @enum State
    *   @brief 探索状態を列挙
    */
    enum State{
      START,									//< 初期位置，初期姿勢
      SEARCHING_FOR_GOAL,			//< ゴール区画を探索中
      SEARCHING_ADDITIONALLY,	//< 追加探索中
      BACKING_TO_START, 			//< スタートに戻っている
      REACHED_START,					//< スタートに戻ってきた
      IMPOSSIBLE,							//< ゴールにだどりつくことができないと判明した
      IDENTIFYING_POSITION,		//< 自己位置同定中
    };
    /** @function stateString
    *   @brief Stateの表示用文字列を返す関数
    */
    static const char* stateString(const enum State s){
      static const char* str[]={
        "start",
        "Searching for Goal",
        "Searching Additionally",
        "Backing to Start",
        "Reached Start",
        "Impossible",
        "Identifying Position",
      };
      return str[s];
    }
    /** @function replaceGoal
    *   @brief ゴール区画を変更する関数
    */
    void replaceGoal(const Vectors& goal) { searchAlgorithm.replaceGoal(goal); }
    /** @function updateCurVecDir
    *   @brief 現在地を更新
    *   @param v 区画座標
    *   @param d 絶対方向
    */
    void updateCurVecDir(const Vector& v, const Dir& d) { curVec = v; curDir = d; }
    /** @function updateWall
    *   @brief 絶対座標絶対方向で壁の1枚更新
    *   @param v 区画座標
    *   @param d 絶対方向
    *   @param b 壁の有無
    */
    bool updateWall(const Vector& v, const Dir& d, const bool left, const bool front, const bool right, const bool back, Dir& nextDir){
      if(state == IDENTIFYING_POSITION){
        return updateWall(idMaze, idWallLogs, v, d, left, front, right, back, nextDir);
      }
      return updateWall(maze, wallLogs, v, d, left, front, right, back, nextDir);
    }
    bool updateWall(Maze& maze, WallLogs& wallLogs, const Vector& v, const Dir& d, const bool left, const bool front, const bool right, const bool back, Dir& nextDir){
      updateWall(maze, wallLogs, v, d+1, left); // left wall
      updateWall(maze, wallLogs, v, d+0, front); // front wall
      updateWall(maze, wallLogs, v, d-1, right); // right wall
      updateWall(maze, wallLogs, v, d+2, back); // back wall

      // 候補の中で行ける方向を探す
      const auto it = std::find_if(nextDirCandidates.begin(), nextDirCandidates.end(), [&](const Dir& dir){
        return maze.canGo(v, dir);
      });
      if(it == nextDirCandidates.end()) return false;
      nextDir = *it;
      return true;
    }
    bool updateWall(Maze& maze, WallLogs& wallLogs, const Vector& v, const Dir& d, const bool& b) const {
      if(!maze.updateWall(v, d, b)) return false; //< 既知壁と食い違いがあった
      wallLogs.push_back(WallLog(v, d, b));
      return true;
    }
    // bool resetLastWall(const int num = 1){
    //   for(int i=0;i<num;i++){
    //     if(wallLogs.empty()) return true;
    //     auto wl = wallLogs.back();
    //     maze.setWall(Vector(wl), wl.d, false);
    //     maze.setKnown(Vector(wl), wl.d, false);
    //     wallLogs.pop_back();
    //   }
    // }
    /** @function calcNextDirs
    *   @brief 次に行くべき方向配列を計算
    *   注意: 処理に時間がかかる場合あり
    *   @return 探索状態
    */
    SearchAlgorithm::Status calcNextDirs(){
      state = START;
      SearchAlgorithm::Status status;
      if(isPositionIdentifying){
        state = IDENTIFYING_POSITION;
        status = searchAlgorithm.calcNextDirsPositionIdentification(idMaze, idWallLogs, curVec, curDir, nextDirs, nextDirCandidates);
        switch(status){
          case SearchAlgorithm::Processing: return status;
          case SearchAlgorithm::Reached: isPositionIdentifying = false; break;
          case SearchAlgorithm::Error: return status;
        }
      }
      state = SEARCHING_FOR_GOAL;
      status = searchAlgorithm.calcNextDirsSearchForGoal(curVec, curDir, nextDirs, nextDirCandidates);
      switch(status){
        case SearchAlgorithm::Processing: return status;
        case SearchAlgorithm::Reached: break;
        case SearchAlgorithm::Error: return status;
      }
      if(!isForceBackToStart){
        state = SEARCHING_ADDITIONALLY;
        status = searchAlgorithm.calcNextDirsSearchAdditionally(curVec, curDir, nextDirs, nextDirCandidates);
        switch(status){
          case SearchAlgorithm::Processing: return status;
          case SearchAlgorithm::Reached: break;
          case SearchAlgorithm::Error: return status;
        }
      }
      state = BACKING_TO_START;
      status = searchAlgorithm.calcNextDirsBackingToStart(curVec, curDir, nextDirs, nextDirCandidates);
      switch(status){
        case SearchAlgorithm::Processing: return status;
        case SearchAlgorithm::Reached: break;
        case SearchAlgorithm::Error: return status;
      }
      state = REACHED_START;
      return status;
    }
    /** @function calcShortestDirs
    *   @brief 最短経路を導出
    *   @param diagonal true: 斜めあり, false: 斜めなし
    *   @return true: 成功, false: 失敗
    */
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
      isPositionIdentifying = true;
      state = IDENTIFYING_POSITION;
      idWallLogs.clear();
      idMaze.reset();
      curVec = SearchAlgorithm::idStartVector();
      curDir = d;
    }
    /** @function getState
    *   @brief 探索状態の取得
    */
    const State& getState() const {
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
    const Dirs& getNextDirCandidates() const {
      return nextDirCandidates;
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
    void printInfo(const bool showMaze = true) const {
      printInfo(showMaze, curVec, curDir, state);
    }
    void printInfo(const bool showMaze, const Vector vec, const Dir dir, const State state) const {
      // 迷路を表示
      if(showMaze) {
        for(int i=0; i<10; i++) printf("\x1b[A"); //< カーソルを移動
        for(int i=0; i<MAZE_SIZE*2; i++) printf("\x1b[A");
        if(isPositionIdentifying) searchAlgorithm.getStepMap().print(idMaze, vec, dir);
        else                      searchAlgorithm.getStepMap().print(maze, vec, dir);
      }
      // 詳細を表示
      printf("Cur: ( %2d, %2d,  %c), State: %s\t\t\t\t\n", vec.x, vec.y, ">^<v"[dir], stateString(state));
      printf("nextDirs: ");
      for (const auto d : getNextDirs()) printf("%c", ">^<v"[d]);
      printf("                                                     \n");
      printf("nextDirCandidates: ");
      for(const auto d: getNextDirCandidates()) printf("%c", ">^<v"[d]);
      printf("        \n");
      printf("Match Count: %d\t\t\t\n", searchAlgorithm.matchCount);
    }
    /** @function printPath
    *   @brief 最短経路の表示
    */
    void printPath() const {
      maze.printPath(Vector(0, 0), shortestDirs);
      printf("Shortest Step: %d\n", (int)shortestDirs.size());
    }
    WallLogs& getWallLog() { return wallLogs; }
    Maze& getMaze() { return maze; }

  private:
    Maze maze; /**< 使用する迷路の参照 */
    Maze idMaze; /**< 使用する迷路の参照 */
    WallLogs wallLogs;
    WallLogs idWallLogs;
    SearchAlgorithm searchAlgorithm; /**< 探索器 */
    State state; /**< 現在の探索状態を保持 */
    Vector curVec; /**< 現在の区画座標 */
    Dir curDir; /**< 現在向いている方向 */
    Dirs nextDirs; /**< 次に行く探索方向配列 */
    Dirs nextDirCandidates; /**< 次に行く方向の候補の優先順 */
    Dirs shortestDirs; /**< 最短経路の方向配列 */
    bool isForceBackToStart = false;
    bool isPositionIdentifying = false;
  };
}
