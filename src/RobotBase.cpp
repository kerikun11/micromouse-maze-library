/**
 * @file RobotBase.cpp
 * @brief ロボットのベース
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-05-20
 * @copyright Copyright 2018 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include "MazeLib/RobotBase.h"

namespace MazeLib {

const char* RobotBase::getSearchActionName(enum SearchAction action) {
  switch (action) {
    case START_STEP:
      return "START_STEP";
    case START_INIT:
      return "START_INIT";
    case ST_FULL:
      return "ST_FULL";
    case ST_HALF:
      return "ST_HARF";
    case ST_HALF_STOP:
      return "ST_HALF_STOP";
    case TURN_L:
      return "TURN_L";
    case TURN_R:
      return "TURN_R";
    case ROTATE_180:
      return "ROTATE_180";
    default:
      return "Unknown";
  }
}
const char* RobotBase::getFastActionName(enum FastAction action) {
  switch (action) {
    case F_ST_FULL:
      return "F_ST_FULL";
    case F_ST_HALF:
      return "F_ST_HALF";
    case F_ST_DIAG:
      return "F_ST_DIAG";
    case F45_L:
      return "F45_L";
    case F45_LP:
      return "F45_LP";
    case F45_R:
      return "F45_R";
    case F45_RP:
      return "F45_RP";
    case F90_L:
      return "F90_L";
    case F90_R:
      return "F90_R";
    case FV90_L:
      return "FV90_L";
    case FV90_R:
      return "FV90_R";
    case FS90_L:
      return "FS90_L";
    case FS90_R:
      return "FS90_R";
    case F135_L:
      return "F135_L";
    case F135_LP:
      return "F135_LP";
    case F135_R:
      return "F135_LP";
    case F135_RP:
      return "F135_RP";
    case F180_L:
      return "F180_L";
    case F180_R:
      return "F180_R";
    default:
      return "Unknown";
  }
}

std::string RobotBase::convertDirectionsToSearchPath(const Directions& dirs) {
  if (dirs.empty())
    return "";
  std::string path;
  path.reserve(dirs.size());
  Direction prevDir = dirs[0];
  for (int i = 1; i < (int)dirs.size(); ++i) {
    const auto nextDir = dirs[i];
    switch (Direction(nextDir - prevDir)) {
      case Direction::Front:
        path += RobotBase::SearchAction::ST_FULL;
        break;
      case Direction::Left:
        path += RobotBase::SearchAction::TURN_L;
        break;
      case Direction::Right:
        path += RobotBase::SearchAction::TURN_R;
        break;
    }
    prevDir = nextDir;
  }
  return path;
}
std::string RobotBase::convertSearchPathToFastPath(std::string src,
                                                   const bool diag_enabled) {
  /* 前後に半分の直線を追加 */
  src = (char)F_ST_HALF + src + (char)F_ST_HALF;
  return replaceStringSearchToFast(src, diag_enabled);
}
std::string RobotBase::convertSearchPathToKnownPath(std::string src,
                                                    const bool diag_enabled) {
  /* 直線を半区画に統一 */
  replace(src, "S", "ss");
  /* 初手ターンを防ぐため、直線を探す */
  auto f = src.find_first_of(F_ST_HALF, 1); /*< 最初の直線を探す */
  auto b = src.find_last_of(F_ST_HALF);     /*< 最後の直線を探す */
  if (f >= b)
    return src; /*< 直線なし */
  /* 前後のターンを除いた、直線に挟まれた区間を抽出 */
  auto fb = src.substr(f, b - f + 1);
  fb = replaceStringSearchToFast(fb, diag_enabled); /*< 最短パターンに変換 */
  /* 最初の直線前と最後の直線後を連結して完了 */
  return src.substr(0, f - 0) + fb + src.substr(b + 1, src.size() - b - 1);
}

void RobotBase::reset() {
  /* 迷路をリセット */
  Agent::reset();
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
  if (!isForceGoingToGoal && isCompleted())
    return true;
  /* 探索中断をクリア */
  setBreakFlag(false);
  /* 自己位置同定をクリア */
  setPositionIdentifying(false);
  /* スタートのアクションをキュー */
  queueAction(START_STEP);
  updateCurrentPose(Pose(Position(0, 1), Direction::North));
  /* 走行開始 */
  return generalSearchRun();
}
bool RobotBase::positionIdentifyRun() {
  /* 探索中断をクリア */
  setBreakFlag(false);
  /* 自己位置同定の初期化 */
  setPositionIdentifying();
  /* 最初のアクションをキュー */
  queueAction(ST_HALF);
  /* 走行開始 */
  return generalSearchRun();
}
bool RobotBase::endFastRunBackingToStartRun() {
  /* エラー処理 */
  if (getShortestDirections().empty()) {
    maze_logw << "ShortestDirections are empty!" << std::endl;
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

void RobotBase::turnbackSave() {
  queueAction(ST_HALF_STOP);
  waitForEndAction();
  stopDequeue();
  if (breakFlag)
    return;
  backupMazeToFlash();
  queueAction(ROTATE_180);
  queueAction(ST_HALF);
  startDequeue();
}
void RobotBase::queueNextDirections(const Directions& nextDirections) {
  for (const auto nextDirection : nextDirections) {
    if (breakFlag)
      return;
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
        maze_loge << "invalid direction" << std::endl;
    }
    updateCurrentPose(current_pose.next(nextDirection));
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
    const auto oldState = getState();
    const auto status = calcNextDirections(); /*< 時間がかかる処理！ */
    const auto newState = getState();
    calcNextDirectionsPostCallback(oldState, newState);
    /* 最短経路導出結果を確認 */
    if (status == SearchAlgorithm::Error) {
      maze_logw << "calcNextDirections Error" << std::endl;
      stopDequeue();
      return false;
    }
    /* 既知区間移動をキューにつめる */
    queueNextDirections(getNextDirectionsKnown());
    /* 走行が終わるのを待つ */
    waitForEndAction();
    /* 探索終了を確認 */
    if (status == SearchAlgorithm::Reached)
      break;
    /* 探索中断を確認 */
    if (breakFlag) {
      maze_logw << "the break flag was set" << std::endl;
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
      maze_logw << "I can't go anywhere!" << std::endl;
      stopDequeue();
      return false;
    }
    queueNextDirections({nextDirection});
  }
  /* スタート区画特有の処理 */
  queueAction(ST_HALF_STOP);
  queueAction(START_INIT);
  updateCurrentPose({Position(0, 0), Direction::North});
  calcNextDirections(); /*< 最終状態に更新 */
  waitForEndAction();
  stopDequeue();
  /* 迷路情報の保存 */
  backupMazeToFlash();
  if (breakFlag) {
    maze_logw << "the break flag was set" << std::endl;
    return false;
  }
  return true;
}

int RobotBase::replace(std::string& src,
                       const std::string& from,
                       const std::string& to) {
  if (from.empty())
    return 0;
  auto pos = src.find(from);
  auto toLen = to.size();
  int i = 0;
  while ((pos = src.find(from, pos)) != std::string::npos) {
    src.replace(pos, from.size(), to);
    pos += toLen;
    i++;
  }
  return i;
}
std::string RobotBase::replaceStringSearchToFast(std::string src,
                                                 bool diag_enabled) {
  replace(src, "S", "ss"); /*< expand */
  replace(src, "L", "ll"); /*< expand */
  replace(src, "R", "rr"); /*< expand */
  if (diag_enabled) {
    replace(src, "rllllr", "rlplr"); /**< FV90 */
    replace(src, "lrrrrl", "lrPrl"); /**< FV90 */
    replace(src, "sllr", "zlr");     /*< F45 */
    replace(src, "srrl", "crl");     /*< F45 */
    replace(src, "rlls", "rlZ");     /*< F45 P */
    replace(src, "lrrs", "lrC");     /*< F45 P */
    replace(src, "sllllr", "alr");   /*< F135 */
    replace(src, "srrrrl", "drl");   /*< F135 */
    replace(src, "rlllls", "rlA");   /*< F135 P */
    replace(src, "lrrrrs", "lrD");   /*< F135 P */
    replace(src, "slllls", "u");     /*< F180 */
    replace(src, "srrrrs", "U");     /*< F180 */
    replace(src, "rllr", "rlwlr");   /*< ST_DIAG */
    replace(src, "lrrl", "lrwrl");   /*< ST_DIAG */
    replace(src, "slls", "q");       /*< F90 */
    replace(src, "srrs", "Q");       /*< F90 */
    replace(src, "rl", "");          /*< cleaning */
    replace(src, "lr", "");          /*< cleaning */
    replace(src, "ss", "S");         /*< ST_FULL */
  } else {
    replace(src, "slllls", "u"); /*< F180 */
    replace(src, "srrrrs", "U"); /*< F180 */
    replace(src, "slls", "q");   /*< F90 */
    replace(src, "srrs", "Q");   /*< F90 */
    replace(src, "ll", "L");     /**< FS90 */
    replace(src, "rr", "R");     /**< FS90 */
  }
  return src;
}

}  // namespace MazeLib
