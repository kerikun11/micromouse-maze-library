#pragma once

#include "SearchAlgorithm.h"

namespace MazeLib{
  class PositionIdentifier {
  public:
    PositionIdentifier() : stepMap(tmpMaze) {
      reset();
    }
    bool reset(){
      findCandidates.push_back(start);
    }
    const Vector& getStart() const { return start; }
    /** @function updateWall
    *   @brief 絶対座標絶対方向で壁の1枚更新
    *   @param v 区画座標
    *   @param d 絶対方向
    *   @param b 壁の有無
    */
    bool updateWall(const Vector& v, const Dir& d, const bool& b){
      // 既知の壁と食い違いがあったら未知壁とする
      if(tmpMaze.isKnown(v, d) && tmpMaze.isWall(v, d) != b){
        tmpMaze.setWall(v, d, false);
        tmpMaze.setKnown(v, d, false);
        return false;
      }
      if(!tmpMaze.isKnown(v, d)){
        tmpMaze.updateWall(v, d, b);
        wallLog.push_back(WallLog(v, d, b));
      }
      return true;
    }
    bool calcNextDirs(const Vector& pv, const Dir& pd, Dirs& nextDirs, Dirs& nextDirsInAdvance) {
      stepMap.update(findCandidates, false, false);
      stepMap.calcNextDirs(pv, pd, nextDirs, nextDirsInAdvance);
    }
    int identify(const Maze& maze, Vector& ans) {
      findCandidates.clear();
      int cnt = 0;
      for(int x=-MAZE_SIZE+1; x<MAZE_SIZE; x++)
      for(int y=-MAZE_SIZE+1; y<MAZE_SIZE; y++) {
        const Vector offset(x, y);
        int diffs=0;
        for(auto wl: wallLog){
          Vector v(wl.x, wl.y);
          Dir d = wl.d;
          if(maze.isKnown(v+offset, d) && maze.isWall(v+offset, d) != wl.b) diffs++;
        }
        if(diffs == 0) {
          cnt++;
          ans = start + offset;
          if(ans.x>=0 && ans.x<MAZE_SIZE && ans.y>=0 && ans.y<MAZE_SIZE) findCandidates.push_back(ans);
        }
      }
      return cnt;
    }
    /** @function getMaze */
    const Maze& getMaze() const {
      return tmpMaze;
    }
		void printMap(const Vector& v, const Dir& d) const {
      for(int i=0; i<MAZE_SIZE*2; i++) printf("\x1b[A");
      stepMap.print(v, d);
    }

  private:
    const Vector start{MAZE_SIZE/2, MAZE_SIZE/2};
    Maze tmpMaze;
    WallLogs wallLog;
    Vectors findCandidates;
    StepMap stepMap;
  };
}
