/**
 * @file RobotBase.cpp
 * @brief ロボットのベース
 * @author KERI (Github: kerikun11)
 * @url https://kerikeri.top/
 * @date 2017.10.30
 */
#include "RobotBase.h"

namespace MazeLib {

bool RobotBase::searchRun() {
  /* 既に探索済みなら正常終了 */
  if (!isForceGoingToGoal && isComplete())
    return true;
  /* ゴール区画への訪問を指定 */
  setForceGoingToGoal();
  /* スタートのアクションをキュー */
  queueAction(START_STEP);
  updateCurVecDir(Vector(0, 1), Dir::North);
  /* 走行開始 */
  return generalSearchRun();
}
bool RobotBase::positionIdentifyRun() {
  /* 自己位置同定の初期化 */
  positionIdentify();
  /* ゴール区画への訪問を指定 */
  setForceGoingToGoal();
  /* 最初のアクションをキュー */
  queueAction(ST_HALF);
  /* 走行開始 */
  return generalSearchRun();
}
bool RobotBase::endFastRunBackingToStartRun() {
  /* エラー処理 */
  if (getShortestDirs().empty()) {
    logw << "ShortestDirs are empty!" << std::endl;
    return false;
  }
  /* 現在位置を最短後の位置に移す */
  Vector v = maze.getStart();
  for (const auto d : getShortestDirs())
    v = v.next(d);
  updateCurVecDir(v, getShortestDirs().back());
  /* 最短後は区画の中央にいるので，区画の切り替わり位置に移動 */
  updateCurVecDir(getCurVec().next(getCurDir() + Dir::Back),
                  getCurDir() + Dir::Back);
  queueAction(ROTATE_180);
  queueAction(ST_HALF);
  /* 走行開始 */
  return generalSearchRun();
}
bool RobotBase::fastRun(const bool diag_enabled) {
  if (!calcShortestDirs(diag_enabled)) {
    loge << "Failed to find shortest path!" << std::endl;
    return false;
  }
  /* fast run here */
  return true;
}

/* private: */

void RobotBase::turnbackSave() {
  queueAction(ST_HALF_STOP);
  waitForEndAction();
  stopDequeue();
  backupMazeToFlash();
  queueAction(ROTATE_180);
  queueAction(ST_HALF);
  startDequeue();
}
void RobotBase::queueNextDirs(const Dirs &nextDirs) {
  for (const auto nextDir : nextDirs) {
    const auto nextVec = getCurVec().next(nextDir);
    switch (Dir(nextDir - getCurDir())) {
    case Dir::Front:
      queueAction(ST_FULL);
      break;
    case Dir::Left:
      queueAction(TURN_L);
      break;
    case Dir::Right:
      queueAction(TURN_R);
      break;
    case Dir::Back:
      turnbackSave();
      break;
    default:
      logw << "invalid direction" << std::endl;
    }
    updateCurVecDir(nextVec, nextDir);
  }
}
bool RobotBase::generalSearchRun() {
  /* スタート前のキャリブレーション */
  calibration();
  /* 走行開始 */
  startDequeue();
  while (1) {
    const auto &v = getCurVec();
    const auto &d = getCurDir();
    /* 既知区間の走行中に最短経路を導出 */
    calcNextDirsPreCallback();
    const auto prevState = getState();
    const auto status = calcNextDirs(); /*< 時間がかかる処理！ */
    const auto newState = getState();
    calcNextDirsPostCallback(prevState, newState);
    /* 既知区間移動をキューにつめる */
    queueNextDirs(getNextDirs());
    /* 最短経路導出結果を確認 */
    if (status == SearchAlgorithm::Reached)
      break;
    if (status == SearchAlgorithm::Error) {
      stopDequeue();
      return false;
    }
    /* 走行が終わるのを待つ */
    waitForEndAction();
    /* 壁を確認 */
    bool left, front, right, back;
    findWall(left, front, right, back);
    if (!updateWall(v, d, left, front, right, back))
      discrepancyWithKnownWall();
    /* 壁のない方向へ1マス移動 */
    Dir nextDir;
    if (!findNextDir(v, d, nextDir)) {
      loge << "I can't go anywhere!" << std::endl;
      stopDequeue();
      return false;
    }
    queueNextDirs({nextDir});
  }
  /* スタート区画特有の処理 */
  queueAction(ST_HALF_STOP);
  queueAction(START_INIT);
  updateCurVecDir(Vector(0, 0), Dir::North);
  calcNextDirs(); /*< 時間がかかる処理！ */
  waitForEndAction();
  stopDequeue();
  backupMazeToFlash();
  return true;
}

} // namespace MazeLib
