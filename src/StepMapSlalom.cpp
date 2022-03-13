/**
 * @file StepMapSlalom.cpp
 * @brief スラロームのコストベースのステップマップを表現するクラス
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-09-11
 * @copyright Copyright 2019 Ryotaro Onuki <kerikun11+github@gmail.com>
 */

#include "MazeLib/StepMapSlalom.h"

#include <algorithm> /*< for std::find_if, etc. */
#include <functional>
#include <iomanip> /*< for std::setw() */
#include <queue>

namespace MazeLib {

/* Index */
StepMapSlalom::Index StepMapSlalom::Index::next(const Direction nd) const {
  switch (getNodeDirection()) {
    case Direction::East:
    case Direction::North:
    case Direction::West:
    case Direction::South:
      return nd.isAlong() ? Index(getPosition().next(nd), nd)
                          : Index(getWallIndex().next(nd), nd);
    case Direction::NorthEast:
      switch (nd) {
        case Direction::East:
          return Index(x + 1, y + 1, nd);
        case Direction::North:
          return Index(x + 1, y + 1, nd);
        case Direction::West:
          return Index(x, y + 1, nd);
        case Direction::South:
          return Index(x + 1, y, nd);
        default:
          return Index(WallIndex(x, y, z).next(nd), nd);
      }
    case Direction::NorthWest:
      switch (nd) {
        case Direction::East:
          return Index(x + 1, y + 1, nd);
        case Direction::North:
          return Index(x, y + 1, nd);
        case Direction::West:
          return Index(x - 1, y + 1, nd);
        case Direction::South:
          return Index(x - 1, y, nd);
        default:
          return Index(WallIndex(x, y, z).next(nd), nd);
      }
    case Direction::SouthWest:
      switch (nd) {
        case Direction::East:
          return Index(x + 1, y - 1, nd);
        case Direction::North:
          return Index(x - 1, y + 1, nd);
        case Direction::West:
          return Index(x - 1, y, nd);
        case Direction::South:
          return Index(x, y - 1, nd);
        default:
          return Index(WallIndex(x, y, z).next(nd), nd);
      }
    case Direction::SouthEast:
      switch (nd) {
        case Direction::East:
          return Index(x + 1, y, nd);
        case Direction::North:
          return Index(x + 1, y + 1, nd);
        case Direction::West:
          return Index(x, y - 1, nd);
        case Direction::South:
          return Index(x + 1, y - 1, nd);
        default:
          return Index(WallIndex(x, y, z).next(nd), nd);
      }
    default:
      maze_loge << "Invalid Direction: " << nd << std::endl;
      return Index(x, y, z, nd);
  }
}
std::ostream& operator<<(std::ostream& os, const StepMapSlalom::Index& i) {
  if (i.getNodeDirection().isAlong())
    return os << "( " << std::setw(2) << (int)i.x << ", " << std::setw(2)
              << (int)i.y << ", " << i.getNodeDirection().toChar() << ")";
  return os << "( " << std::setw(2) << (int)i.x << ", " << std::setw(2)
            << (int)i.y << ", " << i.getDirection().toChar() << ", "
            << i.getNodeDirection().toChar() << ")";
}

/* StepMapSlalom */

bool StepMapSlalom::calcShortestDirections(const Maze& maze,
                                           const EdgeCost& edge_cost,
                                           Directions& shortest_dirs,
                                           const bool known_only,
                                           const bool diag_enabled) {
  const auto dest = convertDestinations(maze.getGoals());
  update(maze, edge_cost, dest, known_only, diag_enabled);
  Indexes path;
  if (!genPathFromMap(path))
    return false;
  shortest_dirs = indexes2directions(path, diag_enabled);
  return true;
}
void StepMapSlalom::update(const Maze& maze,
                           const EdgeCost& edge_cost,
                           const Indexes& dest,
                           const bool known_only,
                           const bool diag_enabled) {
  /* 全ノードのコストを最大値に設定 */
  const auto cost = CostMax;
  cost_map.fill(cost);
  /* 更新予約のキュー */
#define STEP_MAP_USE_PRIORITY_QUEUE 0
#if STEP_MAP_USE_PRIORITY_QUEUE == 1
  std::function<bool(const Index, const Index)> greater = [&](const Index i1,
                                                              const Index i2) {
    return cost_map[i1.getIndex()] > cost_map[i2.getIndex()];
  };
  std::priority_queue<Index, std::vector<Index>, decltype(greater)> q(greater);
#else
  std::queue<Index> q;
#endif
  /* dest のコストを0とする */
  for (const auto i : dest) {
    cost_map[i.getIndex()] = 0;
    q.push(i);
  }
  /* known_only を考慮した壁の判定式を用意 */
  const auto canGo = [&](const WallIndex& i) {
    if (maze.isWall(i) || (known_only && !maze.isKnown(i)))
      return false;
    return true;
  };
  /* 更新がなくなるまで更新 */
  while (!q.empty()) {
#if STEP_MAP_USE_PRIORITY_QUEUE
    const auto&& focus = std::move(q.top());
#else
    const auto&& focus = std::move(q.front());
#endif
    q.pop();
    const auto focus_cost = cost_map[focus.getIndex()];
    /* キューに追加する関数を用意 */
    const auto pushAndContinue = [&](const Index& next,
                                     const cost_t edge_cost) {
      const auto next_cost = focus_cost + edge_cost;
      const auto next_index = next.getIndex();
      if (cost_map[next_index] <= next_cost)
        return false;
      cost_map[next_index] = next_cost;
      from_map[next_index] = focus;
      q.push(next);
      return true;
    };
    const auto nd = focus.getNodeDirection();
    if (nd.isAlong()) { /* 区画の中央 */
      /* 直前の壁 */
      if (!canGo(focus))
        continue;
      /* 直進で行けるところまで行く */
      int8_t n = 1;
      for (auto i = focus; canGo(i); ++n) {
        const auto next = i.next(nd);
        if (!pushAndContinue(next, edge_cost.getEdgeCostAlong(n)))
          break;
        i = next;
      }
      if (diag_enabled) {
        /* ターン */
        for (const auto nd_rel_45 : {Direction::Left45, Direction::Right45}) {
          const auto d45 = nd + nd_rel_45;
          const auto d90 = nd + nd_rel_45 * 2;
          const auto d135 = nd + nd_rel_45 * 3;
          const auto d180 = nd + nd_rel_45 * 4;
          /* 横壁 */
          const auto i45 = focus.next(d45);
          if (canGo(i45)) {
            /* 45 */
            if (canGo(i45.next(i45.getNodeDirection())))
              pushAndContinue(i45, edge_cost.getEdgeCostSlalom(F45));
            /* 90 */
            const auto v90 = focus.getPosition().next(nd).next(d90);
            pushAndContinue(Index(v90, d90), edge_cost.getEdgeCostSlalom(F90));
            /* 135 and 180 */
            const auto i135 = i45.next(d135);
            if (canGo(i135)) {
              /* 135 */
              if (canGo(i135.next(i135.getNodeDirection())))
                pushAndContinue(i135, edge_cost.getEdgeCostSlalom(F135));
              /* 180 */
              pushAndContinue(Index(v90.next(d180), d180),
                              edge_cost.getEdgeCostSlalom(F180));
            }
          }
        }
      } else {
        /* 斜めなしのターン */
        const auto p_f = focus.getPosition().next(nd);  //< i.e. vector front
        for (const auto d90 : {nd + Direction::Left, nd + Direction::Right})
          if (canGo(WallIndex(p_f, d90)))  //< 90度方向の壁
            pushAndContinue(Index(p_f, d90), edge_cost.getEdgeCostSlalom(FS90));
      }
    } else { /* 壁の中央（斜めありの場合しかありえない） */
      /* 直前の壁 */
      const auto i_f = focus.next(nd);
      if (!canGo(i_f)) {
        maze_loge << "FWE: " << focus << std::endl;
        continue;
      }
      /* 直進で行けるところまで行く */
      int8_t n = 1;
      for (auto i = i_f;; ++n) {
        const auto next = i.next(nd);
        if (!canGo(next))
          break;
        if (!pushAndContinue(i, edge_cost.getEdgeCostDiag(n)))
          break;
        i = next;
      }
      /* ターン */
      auto nd_r45 = focus.getRelativeDirectionDiagToAlong();
      auto d45 = nd + nd_r45;
      auto d90 = nd + nd_r45 * 2;
      auto d135 = nd + nd_r45 * 3;
      /* 45R */
      pushAndContinue(focus.next(d45), edge_cost.getEdgeCostSlalom(F45));
      /* V90, 135R */
      const auto i90 = i_f.next(d90);
      if (canGo(i90)) {
        /* V90 */
        if (canGo(i90.next(i90.getNodeDirection())))
          pushAndContinue(i90, edge_cost.getEdgeCostSlalom(FV90));
        /* 135 R */
        pushAndContinue(focus.next(d135), edge_cost.getEdgeCostSlalom(F135));
      }
    }
  }
}
bool StepMapSlalom::genPathFromMap(Indexes& path) const {
  path.clear();
  auto i = index_start.opposite();
  while (1) {
    path.push_back(i.opposite());
    if (cost_map[i.getIndex()] == 0)
      break;
    if (cost_map[i.getIndex()] <= cost_map[from_map[i.getIndex()].getIndex()])
      return false;
    i = from_map[i.getIndex()];
  }
  return true;
}
void StepMapSlalom::print(const Maze& maze,
                          const Indexes& indexes,
                          std::ostream& os) const {
  const auto exists = [&](const Index& i) {
    return std::find_if(indexes.cbegin(), indexes.cend(), [&](const Index& ii) {
             if (i.getNodeDirection().isAlong() !=
                 ii.getNodeDirection().isAlong())
               return false;
             if (i.getNodeDirection().isDiag())
               return i.getWallIndex() == ii.getWallIndex();
             return i.getPosition() == ii.getPosition();
           }) != indexes.cend();
  };
  for (int8_t y = MAZE_SIZE - 1; y >= -1; --y) {
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      os << "+";
      if (exists(Index(x, y, 1, Direction::NorthEast)))
        os << C_YE << " X " << C_NO;
      else
        os << (maze.isKnown(x, y, Direction::North)
                   ? (maze.isWall(x, y, Direction::North) ? "---" : "   ")
                   : (C_RE " . " C_NO));
    }
    os << "+" << std::endl;
    if (y != -1) {
      os << '|';
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        if (exists(Index(x, y, 0, Direction::East)))
          os << C_YE << " X " << C_NO;
        else
          os << "   ";
        if (exists(Index(x, y, 0, Direction::NorthEast)))
          os << C_YE << "X" << C_NO;
        else
          os << (maze.isKnown(x, y, Direction::East)
                     ? (maze.isWall(x, y, Direction::East) ? "|" : " ")
                     : (C_RE "." C_NO));
      }
      os << std::endl;
    }
  }
}
StepMapSlalom::Indexes StepMapSlalom::convertDestinations(
    const Positions& src) {
  Indexes dest;
  for (const auto p : src)
    for (const auto nd : Direction::Along4)
      dest.push_back(Index(p, nd));
  return dest;
}
Directions StepMapSlalom::indexes2directions(const Indexes& path,
                                             const bool diag_enabled) {
  if (!diag_enabled) {
    Directions dirs;
    for (int i = 1; i < (int)path.size(); ++i) {
      const auto nd = path[i].getNodeDirection();
      const auto p = path[i - 1].getPosition() - path[i].getPosition();
      for (int j = 0; j < std::abs(p.x) + std::abs(p.y); ++j)
        dirs.push_back(nd);
    }
    return dirs;
  }
  Directions dirs;
  for (int i = 0; i < (int)path.size() - 1; ++i) {
    const auto nd = path[i].getNodeDirection();
    const auto rel_p = path[i + 1].getPosition() - path[i].getPosition();
    const auto rel_nd =
        Direction(path[i + 1].getNodeDirection() - path[i].getNodeDirection());
    if (nd.isAlong()) {
      switch (rel_nd) {
        case Direction::Front:
          for (int j = 0; j < std::abs(rel_p.x) + std::abs(rel_p.y); ++j)
            dirs.push_back(nd);
          break;
        case Direction::Left45:
          dirs.push_back(nd);
          dirs.push_back(nd + Direction::Left);
          break;
        case Direction::Right45:
          dirs.push_back(nd);
          dirs.push_back(nd + Direction::Right);
          break;
        case Direction::Left:
          dirs.push_back(nd);
          dirs.push_back(nd + Direction::Left);
          break;
        case Direction::Right:
          dirs.push_back(nd);
          dirs.push_back(nd + Direction::Right);
          break;
        case Direction::Left135:
          dirs.push_back(nd);
          dirs.push_back(nd + Direction::Left);
          dirs.push_back(nd + Direction::Back);
          break;
        case Direction::Right135:
          dirs.push_back(nd);
          dirs.push_back(nd + Direction::Right);
          dirs.push_back(nd + Direction::Back);
          break;
        case Direction::Back:
          dirs.push_back(nd);
          if (rel_p.rotate(-nd).y > 0) {
            dirs.push_back(nd + Direction::Left);
            dirs.push_back(nd + Direction::Back);
          } else {
            dirs.push_back(nd + Direction::Right);
            dirs.push_back(nd + Direction::Back);
          }
          break;
        default:
          maze_loge << "invalid Direction" << std::endl;
          break;
      }
    } else {
      switch (rel_nd) {
        case Direction::Front:
          for (auto index = path[i]; index != path[i + 1];
               index = index.next(index.getNodeDirection())) {
            const auto nd_45 = index.getRelativeDirectionDiagToAlong();
            dirs.push_back(index.getNodeDirection() + nd_45);
          }
          break;
        case Direction::Left45:
          dirs.push_back(nd + Direction::Left45);
          break;
        case Direction::Right45:
          dirs.push_back(nd + Direction::Right45);
          break;
        case Direction::Left:
          /* V90 */
          dirs.push_back(nd + Direction::Left45);
          dirs.push_back(nd + Direction::Left135);
          break;
        case Direction::Right:
          /* V90 */
          dirs.push_back(nd + Direction::Right45);
          dirs.push_back(nd + Direction::Right135);
          break;
        case Direction::Left135:
          dirs.push_back(nd + Direction::Left45);
          dirs.push_back(nd + Direction::Left135);
          break;
        case Direction::Right135:
          dirs.push_back(nd + Direction::Right45);
          dirs.push_back(nd + Direction::Right135);
          break;
        default:
          maze_loge << "invalid Direction" << std::endl;
          break;
      }
    }
  }
  return dirs;
}

}  // namespace MazeLib
