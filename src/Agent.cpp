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
  /* 迷路を表示 */
  if (showMaze) {
    std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
    searchAlgorithm.printMap(state, vec, dir);
  }
  /* 詳細を表示 */
  std::cout << "Cur: " << VecDir{vec, dir}
            << ", State: " << SearchAlgorithm::stateString(state) << std::endl;
  std::cout << "\x1b[0K"; /*< カーソルの後ろを削除 */
  std::cout << "nextDirsKnown: ";
  for (const auto d : getNextDirs())
    std::cout << d.toChar();
  std::cout << "    " << std::endl;
  std::cout << "\x1b[0K"; /*< カーソルの後ろを削除 */
  std::cout << "nextDirCandidates: ";
  for (const auto d : getNextDirCandidates())
    std::cout << d.toChar();
  std::cout << std::endl;
  std::cout << "Match Count: " << matchCount << "    " << std::endl;
}

} // namespace MazeLib
