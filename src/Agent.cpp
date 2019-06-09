/**
 * @file Agent.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 現在位置，探索状態を管理するクラスを定義するファイル
 * @version 0.1
 * @date 2019-05-09
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "Agent.h"

namespace MazeLib {

void Agent::printInfo(const bool showMaze, const Vector vec, const Dir dir,
                      const SearchAlgorithm::State state) const {
  // 迷路を表示
  if (showMaze) {
    std::cout << "\e[0;0H"; //< カーソルを左上に移動
    searchAlgorithm.printMap(state, vec, dir);
  }
  // 詳細を表示
  std::printf("Cur: ( %2d, %2d, %2c), State: %s \n", vec.x, vec.y, dir.toChar(),
              SearchAlgorithm::stateString(state));
  std::printf("nextDirsKnown: ");
  for (const auto d : getNextDirs())
    std::printf("%c", d.toChar());
  std::printf(" \n");
  std::printf("nextDirCandidates: ");
  for (const auto d : getNextDirCandidates())
    std::printf("%c", d.toChar());
  std::printf("\n");
  std::printf("Match Count: %d   \n", matchCount);
}

} // namespace MazeLib
