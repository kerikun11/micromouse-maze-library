/**
 * @file Agent.cpp
 * @brief 現在位置、探索状態を管理するクラスを定義するファイル
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2018-05-20
 * @copyright Copyright 2018 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include "MazeLib/Agent.h"

namespace MazeLib {

void Agent::printInfo(const bool showMaze) const {
  /* 迷路を表示 */
  if (showMaze) {
    std::cout << "\e[0;0H"; /*< アニメーションのためにカーソルを左上に移動 */
    searchAlgorithm.printStepMap(getState(), getCurrentPose());
  }
  /* 詳細を表示 */
  std::cout << "\e[J"; /*< カーソル以下を消去 */
  std::cout << "Pose: " << getCurrentPose() << ", "
            << "State: " << SearchAlgorithm::getStateString(getState()) << ", "
            << "Force Start: "
            << (calcData.isForceBackToStart ? "true " : "false") << ", "
            << "Force Goal: "
            << (calcData.isForceGoingToGoal ? "true " : "false") << ", "
            << "Unknown Accel: " << (getUnknownAccelFlag() ? "true " : "false")
            << std::endl;
  std::cout << "Known: " << getNextDirectionsKnown() << std::endl;
  std::cout << "Candidates: " << getNextDirectionCandidates() << std::endl;
  if (getState() == SearchAlgorithm::IDENTIFYING_POSITION)
    std::cout << "Match Count: \t" << getMatchCount() << std::endl;
}

}  // namespace MazeLib
