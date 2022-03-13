/**
 * @file main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 迷路探索アルゴリズムの使用例
 * @date 2019-08-29
 */

/*
 * 迷路ライブラリの読み込み
 */
#include "MazeLib/Maze.h"
#include "MazeLib/StepMap.h"

/*
 * 標準ライブラリの読み込み
 */
#include <algorithm>  //< for std::find
#include <thread>     //< for std::this_thread::sleep_for

/**
 * @brief 名前空間の展開
 */
using namespace MazeLib;

/**
 * @brief ロボットを動かす模擬関数
 * @param relative_dir 移動方向(自己位置に対する相対方向)
 */
void MoveRobot(const Direction relative_dir) {
  switch (relative_dir) {
    case Direction::Front:
      return /* <直進の処理> */;
    case Direction::Left:
      return /* <左ターンの処理> */;
    case Direction::Right:
      return /* <右ターンの処理> */;
    case Direction::Back:
      return /* <引き返しの処理> */;
    default:
      maze_loge << "invalid direction: " << relative_dir << std::endl;
      return;
  }
}

/**
 * @brief アニメーション状に迷路を表示する関数
 * @param step_map 表示するステップマップ
 * @param maze 表示する迷路
 * @param pos 迷路上の位置
 * @param dir 進行方向
 */
void ShowAnimation(const StepMap& step_map,
                   const Maze& maze,
                   const Position& pos,
                   const Direction& dir) {
  std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
  step_map.print(maze, pos, dir);
  std::cout << "Searching for goal" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

/**
 * @brief 探索走行のアルゴリズム
 */
int SearchRun(Maze& maze, const Maze& maze_target) {
  /* 探索テスト */
  StepMap step_map;  //< 経路導出に使用するステップマップ
  /* 現在方向は，現在区画に向かう方向を表す．
   * 現在区画から出る方向ではないことに注意する．
   * +---+---+---+ 例
   * |   <       | <--- (0, 2, West)
   * +   +---+ ^ + <--- (2, 2, North)
   * |   >       | <--- (1, 1, East)
   * +   +---+ v + <--- (2, 0, South)
   * | S |       | <--- (0, 0)
   * +---+---+---+
   */
  Position current_pos = Position(0, 0);     //< 現在の区画位置
  Direction current_dir = Direction::North;  //< 現在向いている方向
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
    const auto& goals = maze.getGoals();
    if (std::find(goals.cbegin(), goals.cend(), current_pos) != goals.cend())
      break;
    /* 現在地からゴールへの移動経路を，未知壁はないものとして導出 */
    const auto move_dirs = step_map.calcShortestDirections(
        maze, current_pos, maze.getGoals(), false, true);
    /* エラー処理 */
    if (move_dirs.empty()) {
      maze_loge << "Failed to Find a path to goal!" << std::endl;
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
      ShowAnimation(step_map, maze, current_pos, current_dir);
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
    /* 現在地から最短候補への移動経路を未知壁はないものとして導出 */
    const auto move_dirs = step_map.calcShortestDirections(
        maze, current_pos, shortest_candidates, false, true);
    /* エラー処理 */
    if (move_dirs.empty()) {
      maze_loge << "Failed to Find a path to goal!" << std::endl;
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
      ShowAnimation(step_map, maze, current_pos, current_dir);
    }
  }
  /* 3. スタート区画へ戻る走行 */
  while (1) {
    /* 現在地のスタート区画判定 */
    if (current_pos == maze.getStart())
      break;
    /* 現在地からスタートへの最短経路を既知壁のみの経路で導出 */
    const auto move_dirs = step_map.calcShortestDirections(
        maze, current_pos, {maze.getStart()}, true, true);
    /* エラー処理 */
    if (move_dirs.empty()) {
      maze_loge << "Failed to Find a path to goal!" << std::endl;
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
      ShowAnimation(step_map, maze, current_pos, current_dir);
    }
  }
  /* 正常終了 */
  return 0;
}

/**
 * @brief 最短走行のアルゴリズム
 */
int ShortestRun(const Maze& maze) {
  /* スタートからゴールまでの最短経路導出 */
  StepMap step_map;
  const auto shortest_dirs = step_map.calcShortestDirections(
      maze, maze.getStart(), maze.getGoals(), true, false);
  if (shortest_dirs.empty()) {
    maze_loge << "Failed to Find a path to goal!" << std::endl;
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
    ShowAnimation(step_map, maze, current_pos, current_dir);
  }
  /* 最短経路の表示 */
  maze.print(shortest_dirs);
  /* 終了 */
  return 0;
}

/**
 * @brief main 関数
 */
int main(void) {
  /* 画面のクリア */
  std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
  std::cout << "\x1b[J";  /*< カーソル以下を消去 */

  /* シミュレーションに用いる迷路の選択 */
  const std::string file_path = "../mazedata/data/16MM2018CX.maze";

  /* 正解の迷路を用意 */
  Maze maze_target;
  /* ファイルから迷路情報を取得 */
  if (!maze_target.parse(file_path.c_str())) {
    std::cerr << "Failed to Parse Maze: " << file_path << std::endl;
    return -1;
  }
  /* 正解の迷路の表示 */
  maze_target.print();

  /* 探索用の迷路を用意 */
  Maze maze;
  /* ゴール位置を設定 */
  maze.setGoals(maze_target.getGoals());

  /* 探索走行テスト */
  SearchRun(maze, maze_target);

  /* 最短走行テスト */
  ShortestRun(maze);

  /* 終了 */
  return 0;
}
