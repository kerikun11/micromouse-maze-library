/**
 * @file main.cpp
 * @brief 迷路探索アルゴリズムの使用例
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
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

/*
 * 名前空間の展開
 */
using namespace MazeLib;

/**
 * @brief ロボットを動かす模擬関数
 * @param relativeDir 移動方向(自己位置に対する相対方向)
 */
void MoveRobot(const Direction relativeDir) {
  switch (relativeDir) {
    case Direction::Front:
      return /* <直進の処理> */;
    case Direction::Left:
      return /* <左ターンの処理> */;
    case Direction::Right:
      return /* <右ターンの処理> */;
    case Direction::Back:
      return /* <引き返しの処理> */;
    default:
      MAZE_LOGE << "invalid direction: " << relativeDir << std::endl;
      return;
  }
}

/**
 * @brief アニメーション状に迷路を表示する関数
 * @param stepMap 表示するステップマップ
 * @param maze 表示する迷路
 * @param pos 迷路上の位置
 * @param dir 進行方向
 */
void ShowAnimation(const StepMap& stepMap,
                   const Maze& maze,
                   const Position& pos,
                   const Direction& dir,
                   const std::string& msg) {
  std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
  stepMap.print(maze, pos, dir);
  std::cout << msg << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

/**
 * @brief 探索走行のアルゴリズム
 */
int SearchRun(Maze& maze, const Maze& mazeTarget) {
  /* 探索テスト */
  StepMap stepMap;  //< 経路導出に使用するステップマップ
  /* 現在方向は、現在区画に向かう方向を表す。
   * 現在区画から出る方向ではないことに注意する。
   * +---+---+---+ 例
   * |   <       | <--- (0, 2, West)
   * +   +---+ ^ + <--- (2, 2, North)
   * |   >       | <--- (1, 1, East)
   * +   +---+ v + <--- (2, 0, South)
   * | S |       | <--- (0, 0)
   * +---+---+---+
   */
  Position currentPos = Position(0, 0);     //< 現在の区画位置
  Direction currentDir = Direction::North;  //< 現在向いている方向
  /* 1. ゴールへ向かう探索走行 */
  while (1) {
    /* 壁を確認。ここでは mazeTarget を参照しているが、実際には壁を見る */
    const bool wall_front =
        mazeTarget.isWall(currentPos, currentDir + Direction::Front);
    const bool wall_left =
        mazeTarget.isWall(currentPos, currentDir + Direction::Left);
    const bool wall_right =
        mazeTarget.isWall(currentPos, currentDir + Direction::Right);
    /* 迷路の壁を更新 */
    maze.updateWall(currentPos, currentDir + Direction::Front, wall_front);
    maze.updateWall(currentPos, currentDir + Direction::Left, wall_left);
    maze.updateWall(currentPos, currentDir + Direction::Right, wall_right);
    /* 現在地のゴール判定 */
    const auto& goals = maze.getGoals();
    if (std::find(goals.cbegin(), goals.cend(), currentPos) != goals.cend())
      break;
    /* 現在地からゴールへの移動経路を、未知壁はないものとして導出 */
    const auto moveDirs = stepMap.calcShortestDirections(
        maze, currentPos, maze.getGoals(), false, true);
    /* エラー処理 */
    if (moveDirs.empty()) {
      MAZE_LOGE << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 未知壁のある区画に当たるまで進む */
    for (const auto nextDir : moveDirs) {
      /* 未知壁があったら終了 */
      if (maze.unknownCount(currentPos))
        break;
      /* ロボットを動かす */
      const auto relativeDir = Direction(nextDir - currentDir);
      MoveRobot(relativeDir);
      /* 現在地を進める */
      currentPos = currentPos.next(nextDir);
      currentDir = nextDir;
      /* アニメーション表示 */
      ShowAnimation(stepMap, maze, currentPos, currentDir,
                    "Searching for Goal");
    }
  }
  /* 2. 最短経路上の未知区画をつぶす探索走行 */
  while (1) {
    /* 壁を確認。ここでは mazeTarget を参照しているが、実際には壁を見る */
    const bool wall_front =
        mazeTarget.isWall(currentPos, currentDir + Direction::Front);
    const bool wall_left =
        mazeTarget.isWall(currentPos, currentDir + Direction::Left);
    const bool wall_right =
        mazeTarget.isWall(currentPos, currentDir + Direction::Right);
    /* 迷路の壁を更新 */
    maze.updateWall(currentPos, currentDir + Direction::Front, wall_front);
    maze.updateWall(currentPos, currentDir + Direction::Left, wall_left);
    maze.updateWall(currentPos, currentDir + Direction::Right, wall_right);
    /* 最短経路上の未知区画を洗い出し */
    const auto shortestDirs = stepMap.calcShortestDirections(
        maze, maze.getStart(), maze.getGoals(), false, false);
    Positions shortestCandidates;
    auto pos = maze.getStart();
    for (const auto nextDir : shortestDirs) {
      pos = pos.next(nextDir);
      if (maze.unknownCount(pos))
        shortestCandidates.push_back(pos);
    }
    /* 最短経路上に未知区画がなければ次へ */
    if (shortestCandidates.empty())
      break;
    /* 現在地から最短候補への移動経路を未知壁はないものとして導出 */
    const auto moveDirs = stepMap.calcShortestDirections(
        maze, currentPos, shortestCandidates, false, true);
    /* エラー処理 */
    if (moveDirs.empty()) {
      MAZE_LOGE << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 未知壁のある区画に当たるまで進む */
    for (const auto nextDir : moveDirs) {
      /* 未知壁があったら終了 */
      if (maze.unknownCount(currentPos))
        break;
      /* ロボットを動かす */
      const auto relativeDir = Direction(nextDir - currentDir);
      MoveRobot(relativeDir);
      /* 現在地を進める */
      currentPos = currentPos.next(nextDir);
      currentDir = nextDir;
      /* アニメーション表示 */
      ShowAnimation(stepMap, maze, currentPos, currentDir,
                    "Searching for Shortest Path Candidates");
    }
  }
  /* 3. スタート区画へ戻る走行 */
  while (1) {
    /* 現在地のスタート区画判定 */
    if (currentPos == maze.getStart())
      break;
    /* 現在地からスタートへの最短経路を既知壁のみの経路で導出 */
    const auto moveDirs = stepMap.calcShortestDirections(
        maze, currentPos, {maze.getStart()}, true, true);
    /* エラー処理 */
    if (moveDirs.empty()) {
      MAZE_LOGE << "Failed to Find a path to goal!" << std::endl;
      return -1;
    }
    /* 経路上を進む */
    for (const auto nextDir : moveDirs) {
      /* ロボットを動かす */
      const auto relativeDir = Direction(nextDir - currentDir);
      MoveRobot(relativeDir);
      /* 現在地を進める */
      currentPos = currentPos.next(nextDir);
      currentDir = nextDir;
      /* アニメーション表示 */
      ShowAnimation(stepMap, maze, currentPos, currentDir,
                    "Going Back to Start");
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
  StepMap stepMap;
  const auto shortestDirs = stepMap.calcShortestDirections(
      maze, maze.getStart(), maze.getGoals(), true, false);
  if (shortestDirs.empty()) {
    MAZE_LOGE << "Failed to Find a path to goal!" << std::endl;
    return -1;
  }
  /* 最短走行 */
  Position currentPos = maze.getStart();
  Direction currentDir = Direction::North;
  for (const auto nextDir : shortestDirs) {
    /* ロボットを動かす */
    const auto relativeDir = Direction(nextDir - currentDir);
    MoveRobot(relativeDir);
    /* 現在地を進める */
    currentPos = currentPos.next(nextDir);
    currentDir = nextDir;
    /* アニメーション表示 */
    ShowAnimation(stepMap, maze, currentPos, currentDir, "Shortest Run");
  }
  /* 最短経路の表示 */
  maze.print(shortestDirs);
  /* 終了 */
  return 0;
}

/**
 * @brief main 関数
 */
int main(void) {
  /* 画面のクリア */
  std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
  std::cout << "\e[J";    /*< カーソル以下を消去 */

  /* シミュレーションに用いる迷路の選択 */
  const std::string filepath = "../mazedata/data/16MM2018CX.maze";

  /* 正解の迷路を用意 */
  Maze mazeTarget;
  /* ファイルから迷路情報を取得 */
  if (!mazeTarget.parse(filepath.c_str())) {
    std::cerr << "Failed to Parse Maze: " << filepath << std::endl;
    return -1;
  }
  /* 正解の迷路の表示 */
  mazeTarget.print();

  /* 探索用の迷路を用意 */
  Maze maze;
  /* ゴール位置を設定 */
  maze.setGoals(mazeTarget.getGoals());

  /* 探索走行テスト */
  SearchRun(maze, mazeTarget);

  /* 最短走行テスト */
  ShortestRun(maze);

  /* 終了 */
  return 0;
}
