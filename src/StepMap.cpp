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

StepMap::StepMap() {
  calcStraightStepTable();
  reset();
}
void StepMap::reset(const step_t step) {
  for (int8_t y = 0; y < MAZE_SIZE; ++y)
    for (int8_t x = 0; x < MAZE_SIZE; ++x)
      setStep(x, y, step); //< ステップをクリア
}
StepMap::step_t StepMap::getStep(const int8_t x, const int8_t y) const {
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
  /* preparation */
  const auto pose = Pose(p.next(d + Direction::Back), d);
  const int maze_size = MAZE_SIZE;
  step_t max_step = 0;
  for (int x = 0; x < MAZE_SIZE; ++x)
    for (int y = 0; y < MAZE_SIZE; ++y)
      if (getStep(x, y) != STEP_MAX)
        max_step = std::max(max_step, getStep(x, y));
  const bool simple = (max_step < 999);
  /* start to draw maze */
  for (int8_t y = maze_size; y >= 0; --y) {
    /* Vertical Wall Line*/
    if (y != maze_size) {
      for (uint8_t x = 0; x <= maze_size; ++x) {
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::West);
        const auto k = maze.isKnown(x, y, Direction::West);
        const auto i = WallIndex(Position(x, y), Direction::West);
        if (i.isInsideOfField() && WallIndex(pose.p, pose.d) == i)
          os << C_YE << pose.d << C_NO;
        else
          os << (k ? (w ? "|" : " ") : (C_RE "." C_NO));
        /* Cell */
        if (x != maze_size) {
          if (getStep(x, y) == STEP_MAX)
            os << C_CY << "999" << C_NO;
          else if (simple)
            os << C_CY << std::setw(3) << getStep(x, y) << C_NO;
          else
            os << C_CY << std::setw(3) << getStep(x, y) / 100 << C_NO;
        }
      }
      os << std::endl;
    }
    /* Horizontal Wall Line*/
    for (uint8_t x = 0; x < maze_size; ++x) {
      /* Pillar */
      os << "+";
      /* Horizontal Wall */
      const auto w = maze.isWall(x, y, Direction::South);
      const auto k = maze.isKnown(x, y, Direction::South);
      const auto i = WallIndex(Position(x, y), Direction::South);
      if (i.isInsideOfField() && WallIndex(pose.p, pose.d) == i)
        os << C_YE << " " << pose.d << " " << C_NO;
      else
        os << (k ? (w ? "---" : "   ") : (C_RE " . " C_NO));
    }
    /* Last Pillar */
    os << "+" << std::endl;
  }
}
void StepMap::print(const Maze &maze, const Directions &dirs,
                    const Position start, std::ostream &os) const {
  /* preparation */
  std::vector<Pose> path;
  Position p = start;
  for (const auto d : dirs)
    path.push_back({p, d}), p = p.next(d);
  const int maze_size = MAZE_SIZE;
  step_t max_step = 0;
  for (int x = 0; x < MAZE_SIZE; ++x)
    for (int y = 0; y < MAZE_SIZE; ++y)
      if (getStep(x, y) != STEP_MAX)
        max_step = std::max(max_step, getStep(x, y));
  const bool simple = (max_step < 999);
  const auto find = [&](const WallIndex i) {
    return std::find_if(path.cbegin(), path.cend(), [&](const Pose pose) {
      return i.isInsideOfField() && WallIndex(pose.p, pose.d) == i;
    });
  };
  /* start to draw maze */
  for (int8_t y = maze_size; y >= 0; --y) {
    if (y != maze_size) {
      for (uint8_t x = 0; x <= maze_size; ++x) {
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::West);
        const auto k = maze.isKnown(x, y, Direction::West);
        const auto it = find(WallIndex(Position(x, y), Direction::West));
        if (it != path.cend())
          os << C_YE << it->d << C_NO;
        else
          os << (k ? (w ? "|" : " ") : (C_RE "." C_NO));
        /* Cell */
        if (x != maze_size) {
          if (getStep(x, y) == STEP_MAX)
            os << C_CY << "999" << C_NO;
          else if (simple)
            os << C_CY << std::setw(3) << getStep(x, y) << C_NO;
          else
            os << C_CY << std::setw(3) << getStep(x, y) / 100 << C_NO;
        }
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < maze_size; ++x) {
      /* Pillar */
      os << "+";
      /* Horizontal Wall */
      const auto w = maze.isWall(x, y, Direction::South);
      const auto k = maze.isKnown(x, y, Direction::South);
      const auto it = find(WallIndex(Position(x, y), Direction::South));
      if (it != path.cend())
        os << C_YE << " " << it->d << " " << C_NO;
      else
        os << (k ? (w ? "---" : "   ") : (C_RE " . " C_NO));
    }
    /* Last Pillar */
    os << "+" << std::endl;
  }
}
void StepMap::printFull(const Maze &maze, const Position p, const Direction d,
                        std::ostream &os) const {
  return printFull(maze, {d}, p.next(d + Direction::Back), os);
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
          const auto p = pose.p;
          const auto d = pose.d;
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
        const auto p = pose.p;
        const auto d = pose.d;
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
void StepMap::update(const Maze &maze, const Positions &dest,
                     const bool known_only, const bool simple) {
  /* 計算を高速化するため，迷路の大きさを制限 */
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
  min_x -= 1, min_y -= 1, max_x += 2, max_y += 2; /*< 外周を許す */
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
    const auto focus = q.front();
    q.pop();
    const auto focus_step = getStep(focus);
    /* 周辺を走査 */
    for (const auto d : Direction::getAlong4()) {
      auto next = focus;
      /* 直線で行けるところまで更新する */
      for (int8_t i = 1; i < MAZE_SIZE; ++i) {
        if (maze.isWall(next, d) || (known_only && !maze.isKnown(next, d)))
          break; /*< 壁あり or 既知壁のみで未知壁 ならば次へ */
        if (next.x > max_x || next.y > max_y || next.x < min_x ||
            next.y < min_y)
          break; /*< 計算を高速化するため展開範囲を制限 */
        next = next.next(d); /*< 移動 */
        /* 直線加速を考慮したステップを算出 */
        const auto next_step = focus_step + (simple ? 1 : step_table[i]);
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
  // if (nextDirectionsKnown.size()) {
  //   const auto d_back = Direction(start_d + Direction::Back);
  //   if (nextDirectionsKnown.front() == d_back) {
  //     const auto wall_backup = maze.isWall(start_p, d_back);
  //     maze.setWall(start_p, d_back, true);
  //     Pose tmp_end;
  //     const auto tmp_nextDirectionsKnown = calcNextDirectionsStepDown(
  //         maze, {start_p, start_d}, tmp_end, false, true);
  //     std::cout << "\e[0;0H"; /*< カーソルを左上に移動 */
  //     maze.print();
  //     getc(stdin);
  //     maze.setWall(start_p, d_back, wall_backup);
  //     if (tmp_nextDirectionsKnown.size()) {
  //       nextDirectionsKnown = tmp_nextDirectionsKnown;
  //       end = tmp_end;
  //     }
  //   }
  // }
  nextDirectionCandidates = calcNextDirectionCandidates(maze, end);
  return end.p;
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
    /* 行く方向を計算しなおす */
    update(maze, dest, false, false);
    calcNextDirections(maze, p, d, tmp_nds, nextDirectionCandidates);
    if (!tmp_nds.empty())
      nextDirectionCandidates = tmp_nds; //< 既知区間になった場合
  }
  /* キャッシュを復活 */
  for (const auto wl : cache) {
    maze.setWall(wl.getPosition(), wl.d, false);
    maze.setKnown(wl.getPosition(), wl.d, false);
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
         {focus.d + Direction::Front, focus.d + Direction::Left,
          focus.d + Direction::Right, focus.d + Direction::Back}) {
      /* 壁あり or 既知壁のみで未知壁 ならば次へ */
      if (maze.isWall(focus.p, d) || (known_only && !maze.isKnown(focus.p, d)))
        continue;
      /* break_unknown で未知壁ならば既知区間は終了 */
      if (break_unknown && !maze.isKnown(focus.p, d))
        return nextDirectionsKnown;
      /* min_step よりステップが小さければ更新 (同じなら更新しない) */
      const auto next = focus.p.next(d);
      const auto next_step = getStep(next);
      if (min_step > next_step) {
        min_step = next_step;
        min_d = d;
      }
    }
    /* focus_step より大きかったらなんかおかしい */
    if (getStep(focus.p) <= min_step)
      break;                              //< 永遠ループ防止
    nextDirectionsKnown.push_back(min_d); //< 既知区間移動
    focus.p = focus.p.next(min_d);        //< 位置を更新
    focus.d = min_d;
  }
  return nextDirectionsKnown;
}
const Directions StepMap::calcNextDirectionCandidates(const Maze &maze,
                                                      const Pose focus) const {
  /* 直線優先で進行方向の候補を抽出．全方位 STEP_MAX だと空になる */
  Directions dirs;
  for (const auto d : {focus.d + Direction::Front, focus.d + Direction::Left,
                       focus.d + Direction::Right, focus.d + Direction::Back})
    if (!maze.isWall(focus.p, d) && getStep(focus.p.next(d)) != STEP_MAX)
      dirs.push_back(d);
  /* ステップが小さい順に並べ替え */
  std::sort(dirs.begin(), dirs.end(),
            [&](const Direction d1, const Direction d2) {
              return getStep(focus.p.next(d1)) < getStep(focus.p.next(d2));
            });
  /* 未知壁優先で並べ替え, これがないと探索時間増大 */
  // std::sort(
  //     dirs.begin(), dirs.end(),
  //     [&](const Direction d1 __attribute__((unused)), const Direction d2) {
  //       return !maze.unknownCount(focus.p.next(d2));
  //     });
  // return dirs;
  Directions tmp_dirs;
  for (const auto d : dirs)
    if (maze.unknownCount(focus.p.next(d)))
      tmp_dirs.push_back(d);
  for (const auto d : dirs)
    if (!maze.unknownCount(focus.p.next(d)))
      tmp_dirs.push_back(d);
  return tmp_dirs;
}
static StepMap::step_t gen_cost_impl(const int i, const float am,
                                     const float vs, const float vm,
                                     const float seg) {
  const auto d = seg * i; /*< i 区画分の走行距離 */
  /* グラフの面積から時間を求める */
  const auto d_thr = (vm * vm - vs * vs) / am; /*< 最大速度に達する距離 */
  if (d < d_thr)
    return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 1000; /*< 三角加速 */
  else
    return (am * d + (vm - vs) * (vm - vs)) / (am * vm) * 1000; /*< 台形加速 */
}
void StepMap::calcStraightStepTable() {
  float vs = 450.0f;         /*< 基本速度 [mm/s] */
  float am_a = 4800.0f;      /*< 最大加速度 [mm/s/s] */
  float vm_a = 1800.0f;      /*< 飽和速度 [mm/s] */
  const float seg_a = 90.0f; /*< 区画の長さ [mm] */
  for (int i = 0; i < MAZE_SIZE; ++i)
    step_table[i] = gen_cost_impl(i, am_a, vs, vm_a, seg_a);
  const step_t turn_cost = 280 - step_table[1];
  for (int i = 0; i < MAZE_SIZE; ++i)
    step_table[i] += turn_cost;
}

} // namespace MazeLib
