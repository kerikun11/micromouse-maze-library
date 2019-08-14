#pragma once

#include "Maze.h"
#include "StepMap.h"

#include <queue>

namespace MazeLib {

class StepMapWall {
public:
public:
  StepMapWall() { calcStraightStepTable(); }
  void reset(const step_t step = MAZE_STEP_MAX) {
    for (int8_t z = 0; z < 2; ++z)
      for (int8_t y = 0; y < MAZE_SIZE; ++y)
        for (int8_t x = 0; x < MAZE_SIZE; ++x)
          setStep(WallIndex(x, y, z), step); //< ステップをクリア
  }
  step_t getStep(const WallIndex i) const {
    return i.isInsideOfFiled() ? stepMap[i] : MAZE_STEP_MAX;
  }
  void setStep(const WallIndex i, const step_t step) {
    if (i.isInsideOfFiled())
      stepMap[i] = step;
  }
  void print(const Maze &maze) const {
    std::ostream &os = std::cout;
    os << std::endl;
    for (int8_t y = MAZE_SIZE; y >= 0; --y) {
      if (y != MAZE_SIZE) {
        os << '|';
        for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
          os << C_CYAN << std::setw(3)
             << std::min(getStep(WallIndex(x, y, 1)), (step_t)999) << C_RESET;
          os << " ";
          os << C_CYAN << std::setw(3)
             << std::min(getStep(WallIndex(x, y, 0)), (step_t)999) << C_RESET;
          os << (maze.isKnown(x, y, Dir::East)
                     ? (maze.isWall(x, y, Dir::East) ? "|" : " ")
                     : (C_RED "." C_RESET));
        }
        os << std::endl;
      }
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        os << "+";
        os << (maze.isKnown(x, y, Dir::South)
                   ? (maze.isWall(x, y, Dir::South) ? "-------" : "       ")
                   : (C_RED "  . .  " C_RESET));
      }
      os << "+" << std::endl;
    }
  }
  void print(const Maze &maze, const Dirs &shortest_dirs,
             const WallIndex start = WallIndex(0, 0, 1)) const {
    auto i = start;
    WallIndexes shortest_indexes;
    shortest_indexes.push_back(i);
    for (const auto d : shortest_dirs) {
      i = i.next(d);
      shortest_indexes.push_back(i);
    }
    print(maze, shortest_indexes);
  }
  void print(const Maze &maze, const WallIndexes &indexes) const {
    std::ostream &os = std::cout;
    os << std::endl;
    for (int8_t y = MAZE_SIZE - 1; y >= -1; --y) {
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        os << "+";
        if (std::find(indexes.cbegin(), indexes.cend(), WallIndex(x, y, 1)) !=
            indexes.cend())
          os << C_YELLOW << " X " << C_RESET;
        else
          os << (maze.isKnown(x, y, Dir::North)
                     ? (maze.isWall(x, y, Dir::North) ? "---" : "   ")
                     : (C_RED " . " C_RESET));
      }
      os << "+" << std::endl;
      if (y != -1) {
        os << '|';
        for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
          os << "   ";
          if (std::find(indexes.cbegin(), indexes.cend(), WallIndex(x, y, 0)) !=
              indexes.cend())
            os << C_YELLOW << "X" << C_RESET;
          else
            os << (maze.isKnown(x, y, Dir::East)
                       ? (maze.isWall(x, y, Dir::East) ? "|" : " ")
                       : (C_RED "." C_RESET));
        }
      }
      os << std::endl;
    }
  }
  void update(const Maze &maze, const WallIndexes &dest,
              const bool known_only) {
    /* 全区画のステップを最大値に設定 */
    reset();
    /* ステップの更新予約のキュー */
    std::queue<WallIndex> q;
    /* destのステップを0とする */
    for (const auto i : dest) {
      setStep(i, 0);
      q.push(i);
    }
    /* ステップの更新がなくなるまで更新処理 */
    while (!q.empty()) {
      /* 注目する区画を取得 */
      const auto focus = q.front();
      q.pop();
      const step_t focus_step = getStep(focus);
      /* 周辺を走査 */
      for (const auto d : focus.getNextDir6()) {
        const auto next = focus.next(d);
        if (known_only && !maze.isKnown(next))
          continue; /*< known_only で未知壁なら更新はしない */
        if (maze.isWall(next))
          continue; /*< 壁があったら更新はしない */
        if (getStep(next) <= focus_step + 1)
          continue;                    /*< 更新の必要がない */
        setStep(next, focus_step + 1); /*< 更新 */
        q.push(next); /*< 再帰的に更新され得るのでキューにプッシュ */
      }
    }
  }
  bool calcShortestDirs(const Maze &maze, const WallIndexes dest,
                        Dirs &shortestDirs, const bool known_only) {
    update(maze, dest, known_only);
    WallIndex end;
    shortestDirs = calcStepDownDirs(maze, WallIndex(0, 0, 1), end, known_only);
    if (getStep(end) == 0)
      return true;
    return false;
  }
  const Vector calcNextDirs(Maze &maze, const Vectors &dest, const Vector vec,
                            const Dir dir, Dirs &nextDirsKnown,
                            Dirs &nextDirCandidates,
                            const bool prior_unknown = true);
  void calcNextDirs(const Maze &maze, const WallIndex start,
                    Dirs &nextDirsKnown, Dirs &nextDirCandidates,
                    const bool prior_unknown = true) const {
    WallIndex known_end;
    /* 既知区間の優先順方向列を作成 */
    nextDirsKnown = calcStepDownDirs(maze, start, known_end, false);
    /* 未知区間の優先順方向列を作成 */
    nextDirCandidates = calcNextDirCandidates(maze, known_end, prior_unknown);
  }

private:
  step_t stepMap[WallIndex::SIZE]; /**< @brief ステップ数*/
  step_t straightStepTable[MAZE_SIZE * 2];

  void calcStraightStepTable() {
    const float a = 6000;
    const float v0 = 300;
    const float factor =
        1.0f / (sqrt(pow(v0 / a, 2) + 90 * (MAZE_SIZE * 2) / a) -
                sqrt(pow(v0 / a, 2) + 90 * (MAZE_SIZE * 2 - 1) / a));
    for (int i = 0; i < MAZE_SIZE * 2; ++i) {
      const float x = 90 * (i + 1);
      straightStepTable[i] = (sqrt(pow(v0 / a, 2) + x / a) - v0 / a) * factor;
    }
  }
  const Dirs calcStepDownDirs(const Maze &maze, const WallIndex start,
                              WallIndex &focus, const bool known_only) const {
    /* ステップマップから既知区間進行方向列を生成 */
    Dirs nextDirsKnown;
    /* start から順にステップマップを下って行く */
    focus = start;
    while (1) {
      /* 周辺の走査; 未知壁の有無と，最小ステップの方向を求める */
      auto min_d = Dir::Max;
      auto min_step = MAZE_STEP_MAX;
      for (const auto d : focus.getNextDir6()) {
        const auto next = focus.next(d);
        /* 未知壁ならば既知区間は終了 */
        if (known_only && !maze.isKnown(next))
          return nextDirsKnown;
        /* 壁があったら次へ */
        if (maze.isWall(next))
          continue;
        /* min_step よりステップが小さければ更新 (同じなら更新しない) */
        const auto next_step = getStep(next);
        if (min_step > next_step) {
          min_step = next_step;
          min_d = d;
        }
      }
      /* focus_step より大きかったらなんかおかしい */
      if (getStep(focus) <= min_step)
        break;                        //< 永遠ループ防止
      nextDirsKnown.push_back(min_d); //< 既知区間移動
      focus = focus.next(min_d);      //< 位置を更新
    }
    return nextDirsKnown;
  }
  const Dirs calcNextDirCandidates(const Maze &maze, const WallIndex focus,
                                   bool prior_unknown) const {
    /* 方向の候補を抽出 */
    Dirs dirs;
    for (const auto d : focus.getNextDir6()) {
      const auto next = focus.next(d);
      if (!maze.isWall(next) && getStep(next) != MAZE_STEP_MAX)
        dirs.push_back(d);
    }
    /* ステップが小さい順に並べ替え */
    std::sort(dirs.begin(), dirs.end(), [&](const Dir d1, const Dir d2) {
      return getStep(focus.next(d1)) <
             getStep(focus.next(d2)); /*< 低コスト優先 */
    });
    /* 未知壁優先で並べ替え, これがないと探索時間増大 */
    if (prior_unknown)
      std::sort(dirs.begin(), dirs.end(),
                [&](const Dir d1 __attribute__((unused)), const Dir d2) {
                  return maze.isKnown(focus.next(d2));
                });
    return dirs;
  }
};

} // namespace MazeLib
