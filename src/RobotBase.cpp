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
  updateCurrentPose(Position(0, 1), Direction::North);
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
  if (getShortestDirections().empty()) {
    logw << "ShortestDirections are empty!" << std::endl;
    return false;
  }
  /* 現在位置を最短後の位置に移す */
  Position p = maze.getStart();
  for (const auto d : getShortestDirections())
    p = p.next(d);
  updateCurrentPose(p, getShortestDirections().back());
  /* 最短後は区画の中央にいるので，区画の切り替わり位置に移動 */
  updateCurrentPose(
      getCurrentPosition().next(getCurrentDirection() + Direction::Back),
      getCurrentDirection() + Direction::Back);
  queueAction(ROTATE_180);
  queueAction(ST_HALF);
  /* 走行開始 */
  return generalSearchRun();
}
bool RobotBase::fastRun(const bool diag_enabled) {
  if (!calcShortestDirections(diag_enabled)) {
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
void RobotBase::queueNextDirections(const Directions &nextDirections) {
  for (const auto nextDirection : nextDirections) {
    const auto nextPosition = getCurrentPosition().next(nextDirection);
    switch (Direction(nextDirection - getCurrentDirection())) {
    case Direction::Front:
      queueAction(ST_FULL);
      break;
    case Direction::Left:
      queueAction(TURN_L);
      break;
    case Direction::Right:
      queueAction(TURN_R);
      break;
    case Direction::Back:
      turnbackSave();
      break;
    default:
      logw << "invalid direction" << std::endl;
    }
    updateCurrentPose(nextPosition, nextDirection);
  }
}
bool RobotBase::generalSearchRun() {
  /* スタート前のキャリブレーション */
  calibration();
  /* 走行開始 */
  startDequeue();
  while (1) {
    const auto &p = getCurrentPosition();
    const auto &d = getCurrentDirection();
    /* 既知区間の走行中に最短経路を導出 */
    calcNextDirectionsPreCallback();
    const auto prevState = getState();
    const auto status = calcNextDirections(); /*< 時間がかかる処理！ */
    const auto newState = getState();
    calcNextDirectionsPostCallback(prevState, newState);
    /* 既知区間移動をキューにつめる */
    queueNextDirections(getNextDirections());
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
    if (!updateWall(p, d, left, front, right, back))
      discrepancyWithKnownWall();
    /* 壁のない方向へ1マス移動 */
    Direction nextDirection;
    if (!findNextDirection(p, d, nextDirection)) {
      loge << "I can't go anywhere!" << std::endl;
      stopDequeue();
      return false;
    }
    queueNextDirections({nextDirection});
  }
  /* スタート区画特有の処理 */
  queueAction(ST_HALF_STOP);
  queueAction(START_INIT);
  updateCurrentPose(Position(0, 0), Direction::North);
  calcNextDirections(); /*< 時間がかかる処理！ */
  waitForEndAction();
  stopDequeue();
  backupMazeToFlash();
  return true;
}

} // namespace MazeLib
