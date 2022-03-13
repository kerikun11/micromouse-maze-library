/**
 * @file Agent.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 現在位置，探索状態を管理するクラスを定義するファイル
 * @copyright Copyright (c) 2018 Ryotaro Onuki
 * @date 2018.05.20
 */
#include "MazeLib/Agent.h"

namespace MazeLib {

void Agent::printInfo(const bool show_maze,
                      const Pose& pose,
                      const SearchAlgorithm::State state) const {
  /* 迷路を表示 */
  if (show_maze) {
    std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
    searchAlgorithm.printMap(state, pose);
  }
  /* 詳細を表示 */
  std::cout << "\x1b[J"; /*< カーソル以下を消去 */
  std::cout << "Pose: " << pose << ", "
            << "State: " << SearchAlgorithm::getStateString(state) << ", "
            << "Force Start: " << (isForceBackToStart ? "true " : "false")
            << ", "
            << "Force Goal: " << (isForceGoingToGoal ? "true " : "false")
            << ", "
            << "Unknown Accel: " << (getUnknownAccelFlag() ? "true " : "false")
            << std::endl;
  std::cout << "Known: ";
  for (const auto d : getNextDirections())
    std::cout << d.toChar();
  std::cout << std::endl;
  std::cout << "Candidates: ";
  for (const auto d : getNextDirectionCandidates())
    std::cout << d.toChar();
  std::cout << std::endl;
  if (state == SearchAlgorithm::IDENTIFYING_POSITION)
    std::cout << "Match Count: \t" << getMatchCount() << std::endl;
}

}  // namespace MazeLib
