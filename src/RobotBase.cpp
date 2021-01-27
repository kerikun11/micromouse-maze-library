/**
 * @file RobotBase.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief ロボットのベース
 * @date 2017.10.30
 */
#include "RobotBase.h"

namespace MazeLib {

void RobotBase::reset() {
  /* 探索中断をクリア */
  setBreakFlag(false);
  /* 自己位置同定をクリア */
  setPositionIdentifying(false);
  /* 強制帰還をクリア */
  setForceBackToStart(false);
  /* ゴール区画への訪問を指定 */
  setForceGoingToGoal();
}
bool RobotBase::searchRun() {
  /* 既に探索済みなら正常終了 */
  if (!isForceGoingToGoal && isComplete())
    return true;
  /* 探索中断をクリア */
  setBreakFlag(false);
  /* スタートのアクションをキュー */
  queueAction(START_STEP);
  updateCurrentPose(Pose(Position(0, 1), Direction::North));
  /* 走行開始 */
  return generalSearchRun();
}
bool RobotBase::positionIdentifyRun() {
  /* 自己位置同定の初期化 */
  setPositionIdentifying();
  /* 探索中断をクリア */
  setBreakFlag(false);
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
  updateCurrentPose({p, getShortestDirections().back()});
  /* 最短後は区画の中央にいるので，区画の切り替わり位置に移動 */
  const auto next_d = getCurrentPose().d + Direction::Back;
  const auto next_p = getCurrentPose().p.next(next_d);
  updateCurrentPose(Pose(next_p, next_d));
  queueAction(ROTATE_180);
  queueAction(ST_HALF);
  /* 探索中断をクリア */
  setBreakFlag(false);
  /* 自己位置同定をクリア */
  setPositionIdentifying(false);
  /* 走行開始 */
  return generalSearchRun();
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
    const auto relative_d = Direction(nextDirection - current_pose.d);
    switch (relative_d) {
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
    updateCurrentPose(current_pose.next(nextDirection));
    if (break_flag)
      return;
  }
}
bool RobotBase::generalSearchRun() {
  /* スタート前のキャリブレーション */
  calibration();
  /* 走行開始 */
  startDequeue();
  while (1) {
    /* 既知区間の走行中に最短経路を導出 */
    calcNextDirectionsPreCallback();
    const auto prevState = getState();
    const auto status = calcNextDirections(); /*< 時間がかかる処理！ */
    const auto newState = getState();
    calcNextDirectionsPostCallback(prevState, newState);
    /* 既知区間移動をキューにつめる */
    queueNextDirections(getNextDirections());
    /* 探索中断を確認 */
    if (break_flag) {
      logi << "the break flag was set" << std::endl;
      stopDequeue();
      return false;
    }
    /* 最短経路導出結果を確認 */
    if (status == SearchAlgorithm::Reached)
      break;
    if (status == SearchAlgorithm::Error) {
      stopDequeue();
      return false;
    }
    /* 走行が終わるのを待つ */
    waitForEndAction();
    /* 探索中断を確認 */
    if (break_flag) {
      logi << "the break flag was set" << std::endl;
      stopDequeue();
      return false;
    }
    /* 壁を確認 */
    bool left, front, right;
    senseWalls(left, front, right);
    if (!updateWall(current_pose, left, front, right))
      discrepancyWithKnownWall();
    /* 壁のない方向へ1マス移動 */
    Direction nextDirection;
    if (!determineNextDirection(current_pose, nextDirection)) {
      loge << "I can't go anywhere! " << std::endl;
      stopDequeue();
      return false;
    }
    queueNextDirections({nextDirection});
  }
  /* スタート区画特有の処理 */
  queueAction(ST_HALF_STOP);
  queueAction(START_INIT);
  updateCurrentPose({Position(0, 0), Direction::North});
  calcNextDirections(); /*< 時間がかかる処理！ */
  waitForEndAction();
  stopDequeue();
  backupMazeToFlash();
  if (break_flag)
    return false;
  return true;
}

} // namespace MazeLib
