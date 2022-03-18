/**
 * @file RobotBase.h
 * @brief ロボットのベース
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-05-20
 * @copyright Copyright 2018 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include "MazeLib/Agent.h"

namespace MazeLib {

/**
 * @brief 迷路を探索ロボットの基底クラス．継承して仮想関数内を埋めて使用する．
 */
class RobotBase : public Agent {
 public:
  enum SearchAction : char {
    START_STEP = '1',
    START_INIT = '2',
    ST_FULL = 'S',
    ST_HALF = 's',
    ST_HALF_STOP = 'E',
    TURN_L = 'L',
    TURN_R = 'R',
    ROTATE_180 = 'T',
  };
  enum FastAction : char {
    F_ST_FULL = 'S',
    F_ST_HALF = 's',
    F_ST_DIAG = 'w',
    F45_L = 'z',
    F45_LP = 'Z',
    F45_R = 'c',
    F45_RP = 'C',
    F90_L = 'q',
    F90_R = 'Q',
    FV90_L = 'p',
    FV90_R = 'P',
    FS90_L = 'L',
    FS90_R = 'R',
    F135_L = 'a',
    F135_LP = 'A',
    F135_R = 'd',
    F135_RP = 'D',
    F180_L = 'u',
    F180_R = 'U',
  };

  static std::string pathConvertSearchToFast(std::string src,
                                             const bool diag_enabled) {
    /* 前後に半分の直線を追加 */
    src = (char)F_ST_HALF + src + (char)F_ST_HALF;
    return replaceStringSearchToFast(src, diag_enabled);
  }
  static std::string pathConvertSearchToKnown(std::string src,
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
  static std::string convertDirectionsToSearchPath(const Directions& dirs) {
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

 public:
  RobotBase() { reset(); }
  void reset();
  bool searchRun();
  bool positionIdentifyRun();
  bool endFastRunBackingToStartRun();
  void setBreakFlag(const bool break_flag = true) {
    this->break_flag = break_flag;
  }

 protected:
  bool break_flag = false; /*< 探索を中断させるフラグ */

  /**
   * @brief 仮想関数．継承して中身を埋める
   */
  virtual void calibration() {}
  virtual void queueAction(const SearchAction action __attribute__((unused))) {}
  virtual void startDequeue() {}
  virtual void waitForEndAction() {}
  virtual void senseWalls(bool& left __attribute__((unused)),
                          bool& front __attribute__((unused)),
                          bool& right __attribute__((unused))) {}
  virtual void stopDequeue() {}
  virtual void calcNextDirectionsPreCallback() {}
  virtual void calcNextDirectionsPostCallback(SearchAlgorithm::State oldState
                                              __attribute__((unused)),
                                              SearchAlgorithm::State newState
                                              __attribute__((unused))) {}
  virtual void discrepancyWithKnownWall() {}
  virtual void backupMazeToFlash() {}

 protected:
  /**
   * @brief 処理関数
   */
  void turnbackSave();
  void queueNextDirections(const Directions& nextDirections);
  bool generalSearchRun();

  /**
   * @brief 文字列置換の汎用関数，
   * 置換対象文字列に含まれる置換前文字列をすべて置換後文字列に置換する
   *
   * @param src 置換対象文字列
   * @param from 置換前文字列
   * @param to 置換後文字列
   * @return int 置換した数
   */
  static int replace(std::string& src,
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
  /**
   * @brief 探索パターンを最短パターンに変換する関数
   *
   * @param src { s, S, L, R} からなる探索パターン文字列
   * @param diag_enabled 斜めありかどうか
   * @return std::string 最短パターン文字列
   */
  static std::string replaceStringSearchToFast(std::string src,
                                               const bool diag_enabled);
};

}  // namespace MazeLib
