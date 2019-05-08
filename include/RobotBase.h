/**
 *  @file RobotBase.h
 *  @brief ロボットのベース
 *  @author KERI (Github: kerikun11)
 *  @url https://kerikeri.top/
 *  @date 2017.10.30
 */
#pragma once

#include "Agent.h"

namespace MazeLib {

class RobotBase : public Agent {
public:
  RobotBase(Maze &maze) : Agent(maze) {}
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
  bool searchRun();
  bool positionIdentifyRun();
  bool endFastRunBackingToStartRun();
  bool fastRun(const bool diagonal);

protected:
  /**
   * @brief 仮想関数．継承して中身を埋める
   */
  virtual void waitForEndAction() {}
  virtual void queueAction(const Action action __attribute__((unused))) {}
  virtual void findWall(bool &left __attribute__((unused)),
                        bool &front __attribute__((unused)),
                        bool &right __attribute__((unused)),
                        bool &back __attribute__((unused))) {}
  virtual void backupMazeToFlash() {}
  virtual void stopDequeue() {}
  virtual void startDequeue() {}
  virtual void calibration() {}
  virtual void calcNextDirsPreCallback() {}
  virtual void calcNextDirsPostCallback(SearchAlgorithm::State prevState
                                        __attribute__((unused)),
                                        SearchAlgorithm::State newState
                                        __attribute__((unused))) {}
  virtual void discrepancyWithKnownWall() {}

private:
  void turnbackSave();
  void queueNextDirs(const Dirs &nextDirs);
  bool generalSearchRun();
};

} // namespace MazeLib
