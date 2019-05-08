/**
 * @file Agent.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief
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
    printf("\e[0;0H"); //< カーソルを左上に移動
    // printf("\e[2J");   //< 画面をクリア (ちらつく)
    searchAlgorithm.printMap(state, vec, dir);
  }
  // 詳細を表示
  printf("Cur: ( %2d, %2d, %2c), State: %s \n", vec.x, vec.y, dir.toChar(),
         SearchAlgorithm::stateString(state));
  printf("nextDirsKnown: ");
  for (const auto d : getNextDirs())
    printf("%c", d.toChar());
  printf(" \n");
  printf("nextDirCandidates: ");
  for (const auto d : getNextDirCandidates())
    printf("%c", d.toChar());
  printf("\n");
  printf("Match Count: %d \n", matchCount);
}

} // namespace MazeLib
