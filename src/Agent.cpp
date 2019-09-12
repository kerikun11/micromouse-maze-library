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

void Agent::printInfo(const bool showMaze, const Pose &pose,
                      const SearchAlgorithm::State state) const {
  /* 迷路を表示 */
  if (showMaze) {
    std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
    searchAlgorithm.printMap(state, pose);
  }
  /* 詳細を表示 */
  std::cout << "\x1b[J"; /*< カーソル以下を消去 */
  std::cout << "Cur: " << pose
            << ", State: " << SearchAlgorithm::getStateString(state)
            << std::endl;
  std::cout << "nextDirectionsKnown: ";
  for (const auto d : getNextDirections())
    std::cout << d.toChar();
  std::cout << "    " << std::endl;
  std::cout << "nextDirectionCandidates: ";
  for (const auto d : getNextDirectionCandidates())
    std::cout << d.toChar();
  std::cout << std::endl;
  std::cout << "Match Count: " << matchCount << std::endl;
}

} // namespace MazeLib
