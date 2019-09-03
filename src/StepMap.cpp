/**
 * @file StepMap.cpp
 * @brief マイクロマウスの迷路のステップマップを扱うクラス
 * @author KERI (Github: kerikun11)
 * @url https://kerikeri.top/
 * @date 2017.11.05
 */
#include "StepMap.h"

#include <algorithm> /*< for std::sort */
#include <cmath>     /*< for std::sqrt, std::pow */
#include <iomanip>   /*< for std::setw() */
#include <queue>

namespace MazeLib {

#define STEP_MAP_USE_PRIORITY_QUEUE 1

StepMap::StepMap() {
  calcStraightStepTable();
  reset();
}
void StepMap::reset(const step_t step) {
  for (int8_t y = 0; y < MAZE_SIZE; ++y)
    for (int8_t x = 0; x < MAZE_SIZE; ++x)
      setStep(x, y, step); //< ステップをクリア
}
step_t StepMap::getStep(const int8_t x, const int8_t y) const {
  /* (x, y) がフィールド内か確認 */
  if (x < 0 || y < 0 || x > MAZE_SIZE - 1 || y > MAZE_SIZE - 1) {
    logw << "referred to out of field: " << Position(x, y) << std::endl;
    return STEP_MAX;
  }
  return step_map[y][x];
}
void StepMap::setStep(const int8_t x, const int8_t y, const step_t step) {
  /* (x, y) がフィールド内か確認 */
  if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) {
    logw << "referred to out of field: " << Position(x, y) << std::endl;
    return;
  }
  step_map[y][x] = step;
}
void StepMap::print(const Maze &maze, const Position p, const Direction d,
                    std::ostream &os) const {
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        os << C_CY << std::setw(3) << std::min(getStep(x, y), (step_t)999)
           << C_NO;
        if ((p == Position(x, y) && d == Direction::West) ||
            (p == Position(x, y).next(Direction::East) && d == Direction::East))
          os << C_YE << d.toChar() << C_NO;
        else
          os << (maze.isKnown(x, y, Direction::East)
                     ? (maze.isWall(x, y, Direction::East) ? "|" : " ")
                     : (C_RE "." C_NO));
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      os << "+";
      if ((p == Position(x, y) && d == Direction::North) ||
          (p == Position(x, y).next(Direction::South) && d == Direction::South))
        os << " " << C_YE << d.toChar() << C_NO << " ";
      else
        os << (maze.isKnown(x, y, Direction::South)
                   ? (maze.isWall(x, y, Direction::South) ? "---" : "   ")
                   : (C_RE " . " C_NO));
    }
    os << "+" << std::endl;
  }
}
void StepMap::print(const Maze &maze, const Directions &dirs,
                    const Position start, std::ostream &os) const {
  uint16_t steps[MAZE_SIZE][MAZE_SIZE] = {{0}};
  Position p = start;
  int counter = 1;
  for (const auto d : dirs) {
    p = p.next(d);
    if (!p.isInsideOfField()) {
      loge << "Out of Field! " << p << std::endl;
      continue;
    }
    steps[p.y][p.x] = counter++;
  }
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (int8_t x = 0; x < MAZE_SIZE; ++x) {
        if (steps[y][x] != 0)
          os << C_YE << std::setw(3) << steps[y][x] << C_NO;
        else
          os << "   ";
        os << (maze.isKnown(x, y, Direction::East)
                   ? (maze.isWall(x, y, Direction::East) ? "|" : " ")
                   : (C_RE "." C_NO));
      }
      os << std::endl;
    }
    for (int8_t x = 0; x < MAZE_SIZE; ++x)
      os << "+"
         << (maze.isKnown(x, y, Direction::South)
                 ? (maze.isWall(x, y, Direction::South) ? "---" : "   ")
                 : (C_RE " . " C_NO));
    os << "+" << std::endl;
  }
}
void StepMap::printFull(const Maze &maze, const Position p, const Direction d,
                        std::ostream &os) const {
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        os << C_CY << std::setw(5) << std::min((int)getStep(x, y), 99999)
           << C_NO;
        if ((p == Position(x, y) && d == Direction::West) ||
            (p == Position(x, y).next(Direction::East) && d == Direction::East))
          os << C_YE << d.toChar() << C_NO;
        else
          os << (maze.isKnown(x, y, Direction::East)
                     ? (maze.isWall(x, y, Direction::East) ? "|" : " ")
                     : (C_RE "." C_NO));
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      os << "+";
      if ((p == Position(x, y) && d == Direction::North) ||
          (p == Position(x, y).next(Direction::South) && d == Direction::South))
        os << "  " << C_YE << d.toChar() << C_NO << "  ";
      else
        os << (maze.isKnown(x, y, Direction::South)
                   ? (maze.isWall(x, y, Direction::South) ? "-----" : "     ")
                   : (C_RE " . . " C_NO));
    }
    os << "+" << std::endl;
  }
}
void StepMap::printFull(const Maze &maze, const Directions &dirs,
                        const Position start, std::ostream &os) const {
  std::vector<Pose> path;
  Position p = start;
  for (const auto d : dirs) {
    p = p.next(d);
    path.push_back({p, d});
  }
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        os << C_CY << std::setw(5) << std::min((int)getStep(x, y), 99999)
           << C_NO;
        bool found = false;
        for (const auto pose : path) {
          const auto p = pose.first;
          const auto d = pose.second;
          if ((p == Position(x, y) && d == Direction::West) ||
              (p == Position(x, y).next(Direction::East) &&
               d == Direction::East)) {
            os << C_YE << d.toChar() << C_NO;
            found = true;
            break;
          }
        }
        if (!found)
          os << (maze.isKnown(x, y, Direction::East)
                     ? (maze.isWall(x, y, Direction::East) ? "|" : " ")
                     : (C_RE "." C_NO));
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      os << "+";
      bool found = false;
      for (const auto pose : path) {
        const auto p = pose.first;
        const auto d = pose.second;
        if ((p == Position(x, y) && d == Direction::North) ||
            (p == Position(x, y).next(Direction::South) &&
             d == Direction::South)) {
          os << "  " << C_YE << d.toChar() << C_NO << "  ";
          found = true;
          break;
        }
      }
      if (!found)
        os << (maze.isKnown(x, y, Direction::South)
                   ? (maze.isWall(x, y, Direction::South) ? "-----" : "     ")
                   : (C_RE " . . " C_NO));
    }
    os << "+" << std::endl;
  }
}
// void StepMap::printFull(const Maze &maze, const Directions &dirs, const
// Position start,
//                         std::ostream &os) const {
//   Positions path;
//   Position p = start;
//   path.push_back(p);
//   for (const auto d : dirs) {
//     p = p.next(d);
//     path.push_back(p);
//   }
//   for (int8_t y = MAZE_SIZE; y >= 0; --y) {
//     if (y != MAZE_SIZE) {
//       os << '|';
//       for (int8_t x = 0; x < MAZE_SIZE; ++x) {
//         if (std::find(path.cbegin(), path.cend(), Position(x, y)) !=
//         path.cend())
//           os << C_YE << std::setw(5) << getStep(x, y) << C_NO;
//         else
//           os << C_CY << std::setw(5) << getStep(x, y) << C_NO;
//         os << (maze.isKnown(x, y, Direction::East)
//                    ? (maze.isWall(x, y, Direction::East) ? "|" : " ")
//                    : (C_RE "." C_NO));
//       }
//       os << std::endl;
//     }
//     for (int8_t x = 0; x < MAZE_SIZE; ++x)
//       os << "+"
//          << (maze.isKnown(x, y, Direction::South)
//                  ? (maze.isWall(x, y, Direction::South) ? "-----" : "     ")
//                  : (C_RE " . . " C_NO));
//     os << "+" << std::endl;
//   }
// }
void StepMap::update(const Maze &maze, const Positions &dest,
                     const bool known_only, const bool simple) {
  /* 迷路の大きさを決定 */
  int8_t min_x = maze.getMinX();
  int8_t max_x = maze.getMaxX();
  int8_t min_y = maze.getMinY();
  int8_t max_y = maze.getMaxY();
  for (const auto p : dest) { /*< ゴールを含めないと導出不可能になる */
    min_x = std::min(p.x, min_x);
    max_x = std::max(p.x, max_x);
    min_y = std::min(p.y, min_y);
    max_y = std::max(p.y, max_y);
  }
  /* 全区画のステップを最大値に設定 */
  reset();
  /* ステップの更新予約のキュー */
  std::queue<Position> q;
  /* destのステップを0とする */
  for (const auto p : dest)
    setStep(p, 0), q.push(p);
  /* ステップの更新がなくなるまで更新処理 */
  while (!q.empty()) {
    /* 注目する区画を取得 */
    const Position focus = q.front();
    q.pop();
    const auto focus_step = getStep(focus);
    /* 周辺を走査 */
    for (const auto d : Direction::getAlong4()) {
      auto next = focus;
      /* 直線で行けるところまで更新する */
      for (int8_t i = 1; i < MAZE_SIZE * 2; ++i) {
        if (maze.isWall(next, d) || (known_only && !maze.isKnown(next, d)))
          break; /*< 壁あり or 既知壁のみで未知壁 ならば次へ */
        if (next.x > max_x + 2 || next.y > max_y + 2 || next.x + 1 < min_x ||
            next.y + 1 < min_y)
          break;             /*< 注目範囲外なら更新しない */
        next = next.next(d); /*< 移動 */
        /* 直線加速を考慮したステップを算出 */
        const auto next_step = focus_step + (simple ? 1 : step_table_along[i]);
        if (getStep(next) <= next_step)
          break;                  /*< 更新の必要がない */
        setStep(next, next_step); /*< 更新 */
        q.push(next); /*< 再帰的に更新され得るのでキューにプッシュ */
        if (simple) /*< 軽量版なら break */
          break;
      }
    }
  }
}
const Directions StepMap::calcShortestDirections(const Maze &maze,
                                                 const Position start,
                                                 const Positions &dest,
                                                 const bool known_only,
                                                 const bool simple) {
  /* ステップマップを更新 */
  update(maze, dest, known_only, simple);
  /* 最短経路となるスタートからの方向列 */
  Directions shortest_dirs;
  /* start から順にステップマップを下る */
  auto focus = start;
  while (1) {
    /* 周辺の走査; 未知壁の有無と，最小ステップの方向を求める */
    auto min_d = Direction::Max;
    auto min_step = STEP_MAX;
    auto min_i = focus;
    /* 周辺を走査 */
    for (const auto d : Direction::getAlong4()) {
      auto next = focus; /*< 隣接 */
      /* 直線で行けるところまで更新する */
      for (int8_t i = 1; i < MAZE_SIZE * 2; ++i) {
        /* 壁があったら次へ */
        if (maze.isWall(next, d) || (known_only && !maze.isKnown(next, d)))
          break;
        next = next.next(d); /*< 移動 */
        /* min_step よりステップが小さければ更新 (同じなら更新しない) */
        const auto next_step = getStep(next);
        if (min_step <= next_step)
          break;
        min_step = next_step;
        min_d = d;
        min_i = next;
      }
    }
    /* focus_step より大きかったらなんかおかしい */
    if (getStep(focus) <= min_step)
      break;
    /* 直線の分だけ移動 */
    while (focus != min_i) {
      focus = focus.next(min_d);      //< 位置を更新
      shortest_dirs.push_back(min_d); //< 既知区間移動
    }
  }
  /* ゴール判定 */
  if (getStep(focus) != 0)
    return {}; //< 失敗
  return shortest_dirs;
}
const Position
StepMap::calcNextDirections(const Maze &maze, const Position start_p,
                            const Direction start_d,
                            Directions &nextDirectionsKnown,
                            Directions &nextDirectionCandidates) const {
  Pose end;
  nextDirectionsKnown =
      calcNextDirectionsStepDown(maze, {start_p, start_d}, end, false, true);
  nextDirectionCandidates = calcNextDirectionCandidates(maze, end);
  return end.first;
}
const Position StepMap::calcNextDirectionsAdv(
    Maze &maze, const Positions &dest, const Position vec, const Direction dir,
    Directions &nextDirectionsKnown, Directions &nextDirectionCandidates) {
  /* ステップマップの更新 */
  update(maze, dest, false, false);
  /* 事前に進む候補を決定する */
  const auto p = calcNextDirections(maze, vec, dir, nextDirectionsKnown,
                                    nextDirectionCandidates);
  Directions ndcs; //< Next Direction Candidates
  WallLogs cache;
  while (1) {
    if (nextDirectionCandidates.empty())
      break;
    const Direction d = nextDirectionCandidates[0]; //< 行きたい方向
    ndcs.push_back(d);                              //< 候補に入れる
    if (maze.isKnown(p, d))
      break;                               //< 既知なら終わり
    cache.push_back(WallLog(p, d, false)); //< 壁をたてるのでキャッシュしておく
    maze.setWall(p, d, true);  //< 壁をたてる
    maze.setKnown(p, d, true); //< 既知とする
    Directions tmp_nds;
    // 行く方向を計算しなおす
    update(maze, dest, false, false);
    calcNextDirections(maze, p, d, tmp_nds, nextDirectionCandidates);
    if (!tmp_nds.empty())
      nextDirectionCandidates = tmp_nds; //< 既知区間になった場合
  }
  // キャッシュを復活
  for (const auto wl : cache) {
    maze.setWall(Position(wl), wl.d, false);
    maze.setKnown(Position(wl), wl.d, false);
  }
  nextDirectionCandidates = ndcs;
  return p;
}
const Directions
StepMap::calcNextDirectionsStepDown(const Maze &maze, const Pose start,
                                    Pose &focus, const bool known_only,
                                    const bool break_unknown) const {
  /* ステップマップから既知区間進行方向列を生成 */
  Directions nextDirectionsKnown;
  /* start から順にステップマップを下って行く */
  focus = start;
  while (1) {
    /* 周辺の走査; 未知壁の有無と，最小ステップの方向を求める */
    auto min_d = Direction::Max;
    auto min_step = STEP_MAX;
    for (const auto d :
         {focus.second + Direction::Front, focus.second + Direction::Left,
          focus.second + Direction::Right, focus.second + Direction::Back}) {
      /* 壁あり or 既知壁のみで未知壁 ならば次へ */
      if (maze.isWall(focus.first, d) ||
          (known_only && !maze.isKnown(focus.first, d)))
        continue;
      /* break_unknown で未知壁ならば既知区間は終了 */
      if (break_unknown && !maze.isKnown(focus.first, d))
        return nextDirectionsKnown;
      /* min_step よりステップが小さければ更新 (同じなら更新しない) */
      const auto next = focus.first.next(d);
      const auto next_step = getStep(next);
      if (min_step > next_step) {
        min_step = next_step;
        min_d = d;
      }
    }
    /* focus_step より大きかったらなんかおかしい */
    if (getStep(focus.first) <= min_step)
      break;                               //< 永遠ループ防止
    nextDirectionsKnown.push_back(min_d);  //< 既知区間移動
    focus.first = focus.first.next(min_d); //< 位置を更新
    focus.second = min_d;
  }
  return nextDirectionsKnown;
}
const Directions StepMap::calcNextDirectionCandidates(const Maze &maze,
                                                      const Pose focus) const {
  /* 方向の候補を抽出 */
  Directions dirs;
  for (const auto d :
       {focus.second + Direction::Front, focus.second + Direction::Left,
        focus.second + Direction::Right, focus.second + Direction::Back}) {
    const auto next = focus.first.next(d);
    if (!maze.isWall(focus.first, d) && getStep(next) != STEP_MAX)
      dirs.push_back(d);
  }
  /* ステップが小さい順に並べ替え */
  std::sort(dirs.begin(), dirs.end(),
            [&](const Direction d1, const Direction d2) {
              return getStep(focus.first.next(d1)) <
                     getStep(focus.first.next(d2)); /*< 低コスト優先 */
            });
  /* 未知壁優先で並べ替え, これがないと探索時間増大 */
  std::sort(
      dirs.begin(), dirs.end(),
      [&](const Direction d1 __attribute__((unused)), const Direction d2) {
        return !maze.unknownCount(focus.first.next(d2));
      });
  return dirs;
}
static step_t gen_cost_impl(const int i, const float am, const float vs,
                            const float vm, const float seg) {
  const auto d = seg * i; /*< i 区画分の走行距離 */
  /* グラフの面積から時間を求める */
  const auto d_thr = (vm * vm - vs * vs) / am; /*< 最大速度に達する距離 */
  if (d < d_thr)
    return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 1000; /*< 三角加速 */
  else
    return (am * d + (vm - vs) * (vm - vs)) / (am * vm) * 1000; /*< 台形加速 */
}
void StepMap::calcStraightStepTable() {
  float vs = 450.0f;    /*< 基本速度 [mm/s] */
  float am_a = 4800.0f; /*< 最大加速度 [mm/s/s] */
  float vm_a = 1800.0f; /*< 飽和速度 [mm/s] */
  const float seg_a = 90.0f;
  for (int i = 0; i < MAZE_SIZE * 2; ++i) {
    step_table_along[i] = gen_cost_impl(i, am_a, vs, vm_a, seg_a);
  }
  const step_t turn_cost = 280 - step_table_along[1];
  for (int i = 0; i < MAZE_SIZE * 2; ++i) {
    step_table_along[i] += turn_cost;
  }
}

} // namespace MazeLib
