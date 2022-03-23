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
  /**
   * @brief 探索の走行パターン一覧。これをキューに詰める。
   */
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
  /**
   * @brief 最短の走行パターン一覧。これをキューに詰める。
   */
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
  static const char* getSearchActionName(enum SearchAction action);
  static const char* getFastActionName(enum FastAction action);
  static std::string convertDirectionsToSearchPath(const Directions& dirs);
  static std::string convertSearchPathToFastPath(std::string src,
                                                 const bool diagEnabled);
  static std::string convertSearchPathToKnownPath(std::string src,
                                                  const bool diagEnabled);

 public:
  RobotBase() { reset(); }
  void reset();
  bool searchRun();
  bool positionIdentifyRun();
  bool endFastRunBackingToStartRun();
  void setBreakFlag(const bool breakFlag = true) {
    this->breakFlag = breakFlag;
  }

 protected:
  /**
   * @brief 仮想関数．継承して中身を埋める
   */
  virtual void calibration() {}
  virtual void queueAction(const SearchAction action __attribute__((unused))) {}
  virtual void startDequeue() {}
  virtual void stopDequeue() {}
  virtual void waitForEndAction() {}
  virtual void backupMazeToFlash() {}
  virtual void discrepancyWithKnownWall() {}
  virtual void senseWalls(bool& left __attribute__((unused)),
                          bool& front __attribute__((unused)),
                          bool& right __attribute__((unused))) {}
  virtual void calcNextDirectionsPreCallback() {}
  virtual void calcNextDirectionsPostCallback(SearchAlgorithm::State oldState
                                              __attribute__((unused)),
                                              SearchAlgorithm::State newState
                                              __attribute__((unused))) {}

 protected:
  bool breakFlag = false; /*< @brief 探索を中断させるフラグ */

  /**
   * @brief 処理関数
   */
  void turnbackSave();
  void queueNextDirections(const Directions& nextDirections);
  bool generalSearchRun();

  /**
   * @brief 文字列置換の汎用関数
   * @details 置換対象文字列に含まれる置換前文字列をすべて置換後文字列に置換する
   *
   * @param src 置換対象文字列
   * @param from 置換前文字列
   * @param to 置換後文字列
   * @return int 置換した数
   */
  static int replace(std::string& src,
                     const std::string& from,
                     const std::string& to);
  /**
   * @brief 探索パターンを最短パターンに変換する関数
   *
   * @param src { s, S, L, R} からなる探索パターン文字列
   * @param diagEnabled 斜めありかどうか
   * @return std::string 最短パターン文字列
   */
  static std::string replaceStringSearchToFast(std::string src,
                                               const bool diagEnabled);
};

}  // namespace MazeLib
