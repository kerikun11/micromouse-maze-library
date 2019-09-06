/**
 * @file main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 迷路探索アルゴリズムの使用例
 * @date 2019-08-29
 */
#include "Maze.h"
#include "StepMap.h"

#include <algorithm> //< for sleep function
#include <thread>    //< for std::find

/**
 * @brief 名前空間の展開
 */
using namespace MazeLib;

/**
 * @brief ロボットを動かす模擬関数
 *
 * @param relative_dir 移動方向(自己位置に対する相対方向)
 */
void MoveRobot(const Direction relative_dir) {
  switch (relative_dir) {
  case Direction::Front:
    /* <直進の処理> */
    break;
  case Direction::Left:
    /* <左ターンの処理> */
    break;
  case Direction::Right:
    /* <右ターンの処理> */
    break;
  case Direction::Back:
    /* <引き返しの処理> */
    break;
  default:
    loge << "invalid direction: " << relative_dir << std::endl;
    break;
  }
}

/**
 * @brief 探索走行のアルゴリズム
 */
int SearchRun(Maze &maze, const Maze &maze_target) {
  /* 探索テスト */
  StepMap step_map; //< 経路導出に使用するステップマップ
  /*
   * 現在方向 (>) は，現在区画 (X) に向かう方向を表す．
   * 現在区画 (X) から出る方向ではないことに注意する．
   * +---+---+---+
   * |   |       |
   * +   +---+   +
   * |   > X     |
   * +---+---+   +
   * |           |
   * +---+---+---+
   */
  Position current_pos = Position(0, 0);    //< 現在の区画位置
  Direction current_dir = Direction::North; //< 現在向いている方向
  /* 1. ゴールへ向かう探索走行 */
  while (1) {
    /* 壁を確認．ここでは maze_target を参照しているが，実際には壁を見る */
    const bool wall_front =
        maze_target.isWall(current_pos, current_dir + Direction::Front);
    const bool wall_left =
        maze_target.isWall(current_pos, current_dir + Direction::Left);
    const bool wall_right =
        maze_target.isWall(current_pos, current_dir + Direction::Right);
    /* 迷路の壁を更新 */
    maze.updateWall(current_pos, current_dir + Direction::Front, wall_front);
    maze.updateWall(current_pos, current_dir + Direction::Left, wall_left);
    maze.updateWall(current_pos, current_dir + Direction::Right, wall_right);
    /* 現在地のゴール判定 */
    const auto &goals = maze.getGoals();
    if (std::find(goals.cbegin(), goals.cend(), current_pos) != goals.cend())
      break;
    /* 現在地からゴールへの最短経路を未知壁はないものとして導出 */
    const auto move_dirs = step_map.calcShortestDirections(
        maze, current_pos, maze.getGoals(), false);
    /* エラー処理 */
    if (move_dirs.empty()) {
      loge << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 未知壁のある区画に当たるまで進む */
    for (const auto next_dir : move_dirs) {
      /* 未知壁があったら終了 */
      if (maze.unknownCount(current_pos))
        break;
      /* ロボットを動かす */
      const auto relative_dir = Direction(next_dir - current_dir);
      MoveRobot(relative_dir);
      /* 現在地を進める */
      current_pos = current_pos.next(next_dir);
      current_dir = next_dir;
      /* アニメーション表示 */
      step_map.print(maze, current_pos, current_dir);
      std::cout << "Searching for goal" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  /* 2. 最短経路上の未知区画をつぶす探索走行 */
  while (1) {
    /* 壁を確認．ここでは maze_target を参照しているが，実際には壁を見る */
    const bool wall_front =
        maze_target.isWall(current_pos, current_dir + Direction::Front);
    const bool wall_left =
        maze_target.isWall(current_pos, current_dir + Direction::Left);
    const bool wall_right =
        maze_target.isWall(current_pos, current_dir + Direction::Right);
    /* 迷路の壁を更新 */
    maze.updateWall(current_pos, current_dir + Direction::Front, wall_front);
    maze.updateWall(current_pos, current_dir + Direction::Left, wall_left);
    maze.updateWall(current_pos, current_dir + Direction::Right, wall_right);
    /* 最短経路上の未知区画を洗い出し */
    const auto shortest_dirs = step_map.calcShortestDirections(
        maze, maze.getStart(), maze.getGoals(), false, false);
    Positions shortest_candidates;
    auto pos = maze.getStart();
    for (const auto next_dir : shortest_dirs) {
      pos = pos.next(next_dir);
      if (maze.unknownCount(pos))
        shortest_candidates.push_back(pos);
    }
    /* 最短経路上に未知区画がなければ次へ */
    if (shortest_candidates.empty())
      break;
    /* 現在地から最短候補への経路を未知壁はないものとして導出 */
    const auto move_dirs = step_map.calcShortestDirections(
        maze, current_pos, shortest_candidates, false);
    /* エラー処理 */
    if (move_dirs.empty()) {
      loge << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 未知壁のある区画に当たるまで進む */
    for (const auto next_dir : move_dirs) {
      /* 未知壁があったら終了 */
      if (maze.unknownCount(current_pos))
        break;
      /* ロボットを動かす */
      const auto relative_dir = Direction(next_dir - current_dir);
      MoveRobot(relative_dir);
      /* 現在地を進める */
      current_pos = current_pos.next(next_dir);
      current_dir = next_dir;
      /* アニメーション表示 */
      step_map.print(maze, current_pos, current_dir);
      std::cout << "Finding shortest path" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  /* 3. スタート区画へ戻る走行 */
  while (1) {
    /* 現在地のスタート区画判定 */
    if (current_pos == maze.getStart())
      break;
    /* 現在地からスタートへの最短経路を既知壁のみの経路で導出 */
    const auto move_dirs = step_map.calcShortestDirections(
        maze, current_pos, {maze.getStart()}, true);
    /* エラー処理 */
    if (move_dirs.empty()) {
      loge << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 経路上を進む */
    for (const auto next_dir : move_dirs) {
      /* ロボットを動かす */
      const auto relative_dir = Direction(next_dir - current_dir);
      MoveRobot(relative_dir);
      /* 現在地を進める */
      current_pos = current_pos.next(next_dir);
      current_dir = next_dir;
      /* アニメーション表示 */
      step_map.print(maze, current_pos, current_dir);
      std::cout << "Going back to start" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  /* スタートからゴールまでの最短経路導出 */
  const bool known_only = true;
  const auto shortest_dirs = step_map.calcShortestDirections(
      maze, maze.getStart(), maze.getGoals(), known_only);
  step_map.printFull(maze, shortest_dirs);
  /* 終了 */
  return 0;
}

/**
 * @brief 最短走行のアルゴリズム
 */
int ShortestRun(const Maze &maze) {
  /* 足立法のステップマップ */
  StepMap step_map;
  const bool known_only = true;
  const auto shortest_dirs = step_map.calcShortestDirections(
      maze, maze.getStart(), maze.getGoals(), known_only, false);
  if (shortest_dirs.empty()) {
    loge << "Failed to Find a path to goal!" << std::endl;
    return -1;
  }
  /* 最短走行 */
  Position current_pos = maze.getStart();
  Direction current_dir = Direction::North;
  for (const auto next_dir : shortest_dirs) {
    /* ロボットを動かす */
    const auto relative_dir = Direction(next_dir - current_dir);
    MoveRobot(relative_dir);
    /* 現在地を進める */
    current_pos = current_pos.next(next_dir);
    current_dir = next_dir;
    /* アニメーション表示 */
    step_map.print(maze, current_pos, current_dir);
    std::cout << "Shortest Run" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  /* 最短経路の表示 */
  step_map.update(maze, {maze.getStart()}, true, false);
  step_map.printFull(maze, shortest_dirs);
  /* 終了 */
  return 0;
}

/**
 * @brief main 関数
 */
int main(void) {
  /* シミュレーションに用いる迷路の選択 */
  const std::string file_path = "../mazedata/16MM2017CX.maze";
  Maze maze_target(file_path.c_str());
  maze_target.print();

  /* 探索用の迷路を用意 */
  Maze maze;
  /* ゴール位置を設定 */
  maze.setGoals(maze_target.getGoals());

  /* 探索走行テスト */
  SearchRun(maze, maze_target);

  /* 最短走行テスト */
  ShortestRun(maze);
  // ShortestRun(maze_target);

  /* 終了 */
  return 0;
}
