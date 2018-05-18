/**
* @file RobotBase.h
* @brief ロボットのベース
* @author KERI (Github: kerikun11)
* @date 2017.10.30
*/
#pragma once

#include "Maze.h"
#include "Agent.h"

namespace MazeLib {
  class RobotBase : public Agent {
  public:
    RobotBase(const Vectors& goal) : Agent(goal) { }
    enum Action : char {
      START_STEP,
      START_INIT,
      STOP_HALF,
      TURN_LEFT_90,
      TURN_RIGHT_90,
      ROTATE_LEFT_90,
      ROTATE_RIGHT_90,
      ROTATE_180,
      STRAIGHT_FULL,
      STRAIGHT_HALF,
    };
    /** @function replaceGoal
    *   @brief ゴール区画を変更する関数
    */
    void replaceGoal(const Vectors& goal) {
      Agent::replaceGoal(goal);
    }
    bool searchRun() {
      if(isComplete()) return true;
      queueAction(START_STEP);
      updateCurVecDir(Vector(0, 1), Dir::North);
      calibration();
      startDequeue();
      auto res = generalSearchRun({start});
      if(!res) {
        stopDequeue();
        return false;
      }
      queueAction(START_INIT);
      updateCurVecDir(Vector(0, 0), Dir::North);
      calcNextDirs(); //< 時間がかかる処理！
      waitForEndAction();
      stopDequeue();
      backupMazeToFlash();
      return true;
    }
    bool positionIdentifyRun(const Dir start_d) {
      positionIdentify(start_d+2);
      queueAction(ROTATE_180);
      queueAction(STRAIGHT_HALF);
      calibration();
      startDequeue();
      auto res = generalSearchRun({start});
      if(!res) {
        stopDequeue();
        return false;
      }
      queueAction(START_INIT);
      updateCurVecDir(Vector(0, 0), Dir::North);
      calcNextDirs(); //< 時間がかかる処理！
      waitForEndAction();
      stopDequeue();
      backupMazeToFlash();
      return true;
    }
    bool endFastRunBackingToStartRun(bool diagonal) {
      queueAction(ROTATE_180);
      queueAction(STRAIGHT_HALF);
      calibration();
      startDequeue();
      auto res = generalSearchRun({start});
      if(!res) {
        stopDequeue();
        return false;
      }
      queueAction(START_INIT);
      updateCurVecDir(Vector(0, 0), Dir::North);
      calcNextDirs(); //< 時間がかかる処理！
      waitForEndAction();
      stopDequeue();
      backupMazeToFlash();
      return true;
    }
    bool fastRun(const bool diagonal){
      if(!calcShortestDirs(diagonal)){
        printf("Failed to find shortest path!\n");
        return false;
      }
      /* move robot here */
      return true;
    }

  protected:
    virtual void waitForEndAction() {}
    virtual void queueAction(const Action action) {}
    virtual bool findWall(const Vector v, const Dir d) { return false; }
    virtual void backupMazeToFlash(){}
    virtual void stopDequeue() {}
    virtual void startDequeue() {}
    virtual void calibration() {}
    virtual void calcNextDirsPreCallback() {}
    virtual void calcNextDirsPostCallback(Agent::State prevState, Agent::State newState) {}

  private:
    void turnbackSave(){
      queueAction(STOP_HALF);
      waitForEndAction();
      stopDequeue();
      backupMazeToFlash();
      queueAction(ROTATE_180);
      queueAction(STRAIGHT_HALF);
      startDequeue();
    }
    void queueNextDirs(const Dirs& nextDirs){
      for(const auto nextDir: nextDirs){
        const auto nextVec = getCurVec().next(nextDir);
        switch (Dir(nextDir - getCurDir())) {
          case Dir::Forward: queueAction(STRAIGHT_FULL); break;
          case Dir::Left:    queueAction(TURN_LEFT_90);  break;
          case Dir::Right:   queueAction(TURN_RIGHT_90); break;
          case Dir::Back:    turnbackSave();             break;
        }
        updateCurVecDir(nextVec, nextDir);
      }
    }
    bool generalSearchRun(const Vectors dest){
      int cnt=0;
      while(1){
        if(cnt++ > 100) return false;

        const auto& v = getCurVec();
        const auto& d = getCurDir();

        calcNextDirsPreCallback();
        Agent::State prevState = getState();
        auto status = calcNextDirs(); //< 時間がかかる処理！
        Agent::State newState = getState();
        calcNextDirsPostCallback(prevState, newState);

        // 既知区間移動をキューにつめる
        queueNextDirs(getNextDirs());

        if(status==SearchAlgorithm::Reached) break;

        waitForEndAction();

        // エラー検出
        if(getNextDirCandidates().empty()) {
          printInfo();
          printf("nextDirCandidates is empty! \n");
          return false;
        }

        // 壁を確認
        if(!updateWall(v, d, findWall(v, d+1), findWall(v, d), findWall(v, d-1), findWall(v, d+2))){
          printInfo();
          printf("There was a discrepancy with known information.\n");
          return false;
        }

        // 壁のない方向へ1マス移動
        Dir nextDir;
        if(!findNextDir(v, d, nextDir)){
          printInfo();
          printf("I can't go anywhere!\n");
          return false;
        }
        queueNextDirs({nextDir});
      }
    }
  };
}
