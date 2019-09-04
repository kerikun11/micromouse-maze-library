/**
 * @file main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 迷路探索アルゴリズムのテストファイル
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
 * @param d_relative 移動方向(自己位置に対する相対方向)
 */
void MoveRobot(const Direction d_relative) {
  switch (d_relative) {
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
    loge << "invalid direction: " << d_relative << std::endl;
    break;
  }
}

/**
 * @brief 最短走行のアルゴリズム
 *
 * @param maze_target
 * @return int
 */
int SearchRun(Maze &maze, const Maze &maze_target) {
  /* 探索テスト */
  StepMap step_map; //< 経路導出に使用するステップマップ
  Position p = Position(0, 0);    //< 現在地
  Direction d = Direction::North; //< 現在向いている方向
  /* ゴールへ向かう */
  while (1) {
    /* 周囲の壁を確認 */
    const bool wall_front = maze_target.isWall(p, d + Direction::Front);
    const bool wall_left = maze_target.isWall(p, d + Direction::Left);
    const bool wall_right = maze_target.isWall(p, d + Direction::Right);
    /* 迷路の壁を更新 */
    maze.updateWall(p, d + Direction::Front, wall_front);
    maze.updateWall(p, d + Direction::Left, wall_left);
    maze.updateWall(p, d + Direction::Right, wall_right);
    /* 現在地のゴール判定 */
    const auto &goals = maze.getGoals();
    if (std::find(goals.cbegin(), goals.cend(), p) != goals.cend())
      break;
    /* 現在地からゴールへの最短経路を未知壁はないものとして導出 */
    const auto next_dirs =
        step_map.calcShortestDirections(maze, p, maze.getGoals(), false);
    /* エラー処理 */
    if (next_dirs.empty()) {
      loge << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 未知壁のある区画に当たるまで進む */
    for (const auto d_next : next_dirs) {
      /* 未知壁があったら終了 */
      if (maze.unknownCount(p))
        break;
      /* ロボットを動かす */
      const auto d_relative = Direction(d_next - d);
      MoveRobot(d_relative);
      /* 現在地を進める */
      p = p.next(d_next);
      d = d_next;
      /* アニメーション表示 */
      step_map.print(maze, p, d);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  /* 最短経路上の未知区画をつぶす */
  while (1) {
    /* 壁を確認．ここでは maze_target を参照しているが，実際には壁を見る */
    const bool wall_front = maze_target.isWall(p, d + Direction::Front);
    const bool wall_left = maze_target.isWall(p, d + Direction::Left);
    const bool wall_right = maze_target.isWall(p, d + Direction::Right);
    /* 迷路の壁を更新 */
    maze.updateWall(p, d + Direction::Front, wall_front);
    maze.updateWall(p, d + Direction::Left, wall_left);
    maze.updateWall(p, d + Direction::Right, wall_right);
    /* 最短経路上の未知区画を洗い出し */
    const auto shortest_dirs = step_map.calcShortestDirections(
        maze, maze.getStart(), maze.getGoals(), false, false);
    Positions shortest_candidates;
    auto v_tmp = maze.getStart();
    for (const auto d_tmp : shortest_dirs) {
      v_tmp = v_tmp.next(d_tmp);
      if (maze.unknownCount(v_tmp))
        shortest_candidates.push_back(v_tmp);
    }
    /* 最短経路上に未知区画がなければ次へ */
    if (shortest_candidates.empty())
      break;
    /* 現在地から最短候補への経路を未知壁はないものとして導出 */
    const auto next_dirs =
        step_map.calcShortestDirections(maze, p, shortest_candidates, false);
    /* エラー処理 */
    if (next_dirs.empty()) {
      loge << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 未知壁のある区画に当たるまで進む */
    for (const auto d_next : next_dirs) {
      /* 未知壁があったら終了 */
      if (maze.unknownCount(p))
        break;
      /* ロボットを動かす */
      const auto d_relative = Direction(d_next - d);
      MoveRobot(d_relative);
      /* 現在地を進める */
      p = p.next(d_next);
      d = d_next;
      /* アニメーション表示 */
      step_map.print(maze, p, d);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  /* スタート区画へ戻る */
  while (1) {
    /* 現在地のスタート区画判定 */
    if (p == maze.getStart())
      break;
    /* 現在地からスタートへの最短経路を既知壁のみの経路で導出 */
    const auto next_dirs =
        step_map.calcShortestDirections(maze, p, {maze.getStart()}, true);
    /* エラー処理 */
    if (next_dirs.empty()) {
      loge << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 経路上を進む */
    for (const auto d_next : next_dirs) {
      /* ロボットを動かす */
      const auto d_relative = Direction(d_next - d);
      MoveRobot(d_relative);
      /* 現在地を進める */
      p = p.next(d_next);
      d = d_next;
      /* アニメーション表示 */
      step_map.print(maze, p, d);
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
 *
 * @param maze
 * @return int
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
  Position p = maze.getStart();
  Direction d = Direction::North;
  for (const auto d_next : shortest_dirs) {
    const auto d_relative = Direction(d_next - d);
    MoveRobot(d_relative);
    /* 現在地を進める */
    p = p.next(d_next);
    d = d_next;
    /* アニメーション表示 */
    step_map.printFull(maze, p, d);
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
  maze.setGoals(maze_target.getGoals());

  /* 探索走行テスト */
  SearchRun(maze, maze_target);

  /* 最短走行テスト */
  ShortestRun(maze);
  // ShortestRun(maze_target);

  /* 終了 */
  return 0;
}
