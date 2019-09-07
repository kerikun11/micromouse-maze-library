/**
 * @file RobotBase.h
 * @brief ロボットのベース
 * @author KERI (Github: kerikun11)
 * @url https://kerikeri.top/
 * @date 2017.10.30
 */
#pragma once

#include "Agent.h"
#include <algorithm> //< std::replace

namespace MazeLib {

/**
 * @brief 迷路を探索ロボットの基底クラス．継承して仮想関数内を埋めて使用する．
 */
class RobotBase : public Agent {
public:
  RobotBase(Maze &maze) : Agent(maze) {}
  enum Action : char {
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
    FL45 = 'z',
    FL45P = 'Z',
    FR45 = 'c',
    FR45P = 'C',
    FL90 = 'q',
    FR90 = 'Q',
    FLV90 = 'p',
    FRV90 = 'P',
    FLS90 = 'L',
    FRS90 = 'R',
    FL135 = 'a',
    FL135P = 'A',
    FR135 = 'd',
    FR135P = 'D',
    FL180 = 'u',
    FR180 = 'U',
  };
  static std::string pathConvertSearchToFast(std::string src,
                                             bool diag_enabled) {
    src = (char)F_ST_HALF + src + (char)F_ST_HALF;
    return replaceStringSearchToFast(src, diag_enabled);
  }
  static std::string pathConvertSearchToKnown(std::string src,
                                              const bool diag_enabled) {
    replace(src, "S", "ss");
    /* 初手ターンを防ぐ */
    auto f = src.find_first_of(F_ST_HALF, 1); /*< 最初の直線を探す */
    auto b = src.find_last_of(F_ST_HALF);     /*< 最後の直線を探す */
    if (f >= b)
      return src;                       /*< 直線なし */
    auto fb = src.substr(f, b - f + 1); /*< 直線に挟まれた区間を抽出 */
    fb = replaceStringSearchToFast(fb,
                                   diag_enabled); /*< 最短走行パターンに変換 */
    /* 最初の直線前と最後の直線後を連結して完了 */
    return src.substr(0, f - 0) + fb + src.substr(b + 1, src.size() - b - 1);
  }
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
  virtual void senseWalls(bool &left __attribute__((unused)),
                          bool &front __attribute__((unused)),
                          bool &right __attribute__((unused))) {}
  virtual void backupMazeToFlash() {}
  virtual void stopDequeue() {}
  virtual void startDequeue() {}
  virtual void calibration() {}
  virtual void calcNextDirectionsPreCallback() {}
  virtual void calcNextDirectionsPostCallback(SearchAlgorithm::State prevState
                                              __attribute__((unused)),
                                              SearchAlgorithm::State newState
                                              __attribute__((unused))) {}
  virtual void discrepancyWithKnownWall() {}

private:
  void turnbackSave();
  void queueNextDirections(const Directions &nextDirections);
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
  static int replace(std::string &src, std::string from, std::string to) {
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
                                               bool diag_enabled) {
    replace(src, "S", "ss");
    replace(src, "L", "ll");
    replace(src, "R", "rr");
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
      replace(src, "rl", "");
      replace(src, "lr", "");
      replace(src, "ss", "S");
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
};

} // namespace MazeLib
