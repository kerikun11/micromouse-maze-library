#include "RobotBase.h"

namespace MazeLib {

bool RobotBase::searchRun() {
  /* 既に探索済みなら正常終了 */
  if (!isForceGoingToGoal && isComplete())
    return true;
  /* ゴール区画への訪問を指定 */
  forceGoingToGoal();
  /* スタートのアクションをキュー */
  queueAction(START_STEP);
  updateCurVecDir(Vector(0, 1), Dir::North);
  /* スタート前のキャリブレーション */
  calibration();
  /* 走行開始 */
  startDequeue();
  auto res = generalSearchRun();
  if (!res) {
    stopDequeue();
    return false;
  }
  /* スタート区画特有の処理 */
  queueAction(START_INIT);
  updateCurVecDir(Vector(0, 0), Dir::North);
  calcNextDirs(); /*< 時間がかかる処理！ */
  waitForEndAction();
  stopDequeue();
  backupMazeToFlash();
  return true;
}
bool RobotBase::positionIdentifyRun() {
  positionIdentify();
  queueAction(STRAIGHT_HALF);
  calibration();
  startDequeue();
  auto res = generalSearchRun();
  if (!res) {
    stopDequeue();
    return false;
  }
  /* スタート区画特有の処理 */
  queueAction(START_INIT);
  updateCurVecDir(Vector(0, 0), Dir::North);
  calcNextDirs(); /*< 時間がかかる処理！ */
  waitForEndAction();
  stopDequeue();
  backupMazeToFlash();
  return true;
}
bool RobotBase::endFastRunBackingToStartRun() {
  Vector v = Vector(0, 0);
  for (auto d : getShortestDirs())
    v = v.next(d);
  updateCurVecDir(v, getShortestDirs().back());
  updateCurVecDir(getCurVec().next(getCurDir() + Dir::Back),
                  getCurDir() + Dir::Back);
  queueAction(ROTATE_180);
  queueAction(STRAIGHT_HALF);
  calibration();
  startDequeue();
  auto res = generalSearchRun();
  if (!res) {
    stopDequeue();
    return false;
  }
  /* スタート区画特有の処理 */
  queueAction(START_INIT);
  updateCurVecDir(Vector(0, 0), Dir::North);
  calcNextDirs(); /*< 時間がかかる処理！ */
  waitForEndAction();
  stopDequeue();
  backupMazeToFlash();
  return true;
}
bool RobotBase::fastRun(const bool diag_enabled) {
  if (!calcShortestDirs(diag_enabled)) {
    std::cerr << "Failed to find shortest path!" << std::endl;
    return false;
  }
  /* move robot here */
  return true;
}

/* private: */

void RobotBase::turnbackSave() {
  queueAction(STOP_HALF);
  waitForEndAction();
  stopDequeue();
  backupMazeToFlash();
  queueAction(ROTATE_180);
  queueAction(STRAIGHT_HALF);
  startDequeue();
}
void RobotBase::queueNextDirs(const Dirs &nextDirs) {
  for (const auto nextDir : nextDirs) {
    const auto nextVec = getCurVec().next(nextDir);
    switch (Dir(nextDir - getCurDir())) {
    case Dir::Front:
      queueAction(STRAIGHT_FULL);
      break;
    case Dir::Left:
      queueAction(TURN_LEFT_90);
      break;
    case Dir::Right:
      queueAction(TURN_RIGHT_90);
      break;
    case Dir::Back:
      turnbackSave();
      break;
    }
    updateCurVecDir(nextVec, nextDir);
  }
}
bool RobotBase::generalSearchRun() {
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
      return true;
    if (status == SearchAlgorithm::Error)
      return false;
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
      std::cerr << "I can't go anywhere!" << std::endl;
      return false;
    }
    queueNextDirs({nextDir});
  }
}

} // namespace MazeLib
