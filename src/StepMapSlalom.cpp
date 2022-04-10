/**
 * @file StepMapSlalom.cpp
 * @brief スラロームのコストベースのステップマップを表現するクラス
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-09-11
 * @copyright Copyright 2019 Ryotaro Onuki <kerikun11+github@gmail.com>
 */

#include "MazeLib/StepMapSlalom.h"

#include <algorithm> /*< for std::find_if */
#include <iomanip>   /*< for std::setw */
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
      MAZE_LOGE << "Invalid Direction: " << nd << std::endl;
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

Directions StepMapSlalom::calcShortestDirections(const Maze& maze,
                                                 const EdgeCost& edgeCost,
                                                 const bool knownOnly) {
  const auto dest = convertDestinations(maze.getGoals());
  update(maze, edgeCost, dest, knownOnly);
  Indexes path;
  if (!genPathFromMap(path))
    return {};
  return indexes2directions(path); /*< 区画ベースの方向列 */
}
void StepMapSlalom::update(const Maze& maze,
                           const EdgeCost& edgeCost,
                           const Indexes& dest,
                           const bool knownOnly) {
  /* 全ノードのコストを最大値に設定 */
  const auto cost = CostMax;
  cost_map.fill(cost);
  from_map.fill(index_start);
  /* 更新予約のキュー */
#define STEP_MAP_USE_PRIORITY_QUEUE 1
#if STEP_MAP_USE_PRIORITY_QUEUE
  struct Element {
    Index v;
    cost_t c;
    bool operator<(const Element& e) const { return c > e.c; }
  };
  std::priority_queue<Element> q;
#else
  std::queue<Index> q;
#endif
  /* dest のコストを0とする */
  for (const auto i : dest)
#if STEP_MAP_USE_PRIORITY_QUEUE
    cost_map[i.getIndex()] = 0, q.push({i, 0});
#else
    cost_map[i.getIndex()] = 0, q.push(i);
#endif
  /* knownOnly を考慮した壁の判定式を用意 */
  const auto canGo = [&](const WallIndex& i) {
    if (maze.isWall(i) || (knownOnly && !maze.isKnown(i)))
      return false;
    return true;
  };
  /* 更新がなくなるまで更新 */
  while (!q.empty()) {
#if MAZE_DEBUG_PROFILING
    queue_size_max = std::max(queue_size_max, q.size());
#endif
#if STEP_MAP_USE_PRIORITY_QUEUE
    const auto focus = q.top().v;
    const auto focus_cost_q = q.top().c;
#else
    const auto focus = q.front();
#endif
    q.pop();
    const auto focus_cost = cost_map[focus.getIndex()];
    /* 枝刈り */
#if STEP_MAP_USE_PRIORITY_QUEUE
    if (focus_cost < focus_cost_q)
      continue;
#endif
    /* キューに追加する関数を用意 */
    const auto pushAndContinue = [&](const Index& next, const cost_t edgeCost) {
      const cost_t next_cost = focus_cost + edgeCost;
      const auto next_index = next.getIndex();
      if (cost_map[next_index] <= next_cost)
        return false;
      cost_map[next_index] = next_cost;
      from_map[next_index] = focus;
#if STEP_MAP_USE_PRIORITY_QUEUE
      q.push({next, next_cost});
#else
      q.push(next);
#endif
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
        if (!pushAndContinue(next, edgeCost.getEdgeCostAlong(n)))
          break;
        i = next;
      }
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
            pushAndContinue(i45, edgeCost.getEdgeCostSlalom(F45));
          /* 90 */
          const auto v90 = focus.getPosition().next(nd).next(d90);
          pushAndContinue(Index(v90, d90), edgeCost.getEdgeCostSlalom(F90));
          /* 135 and 180 */
          const auto i135 = i45.next(d135);
          if (canGo(i135)) {
            /* 135 */
            if (canGo(i135.next(i135.getNodeDirection())))
              pushAndContinue(i135, edgeCost.getEdgeCostSlalom(F135));
            /* 180 */
            pushAndContinue(Index(v90.next(d180), d180),
                            edgeCost.getEdgeCostSlalom(F180));
          }
        }
      }
    } else { /* 壁の中央 */
      /* 直前の壁 */
      const auto i_f = focus.next(nd);
      if (!canGo(i_f)) {
        MAZE_LOGE << "Front Wall Exists: " << focus << std::endl;
        continue;
      }
      /* 直進で行けるところまで行く */
      int8_t n = 1;
      for (auto i = i_f;; ++n) {
        const auto next = i.next(nd);
        if (!canGo(next))
          break;
        if (!pushAndContinue(i, edgeCost.getEdgeCostDiag(n)))
          break;
        i = next;
      }
      /* ターン */
      auto nd_r45 = focus.getRelativeDirectionDiagToAlong();
      auto d45 = nd + nd_r45;
      auto d90 = nd + nd_r45 * 2;
      auto d135 = nd + nd_r45 * 3;
      /* 45R */
      pushAndContinue(focus.next(d45), edgeCost.getEdgeCostSlalom(F45));
      /* V90, 135R */
      const auto i90 = i_f.next(d90);
      if (canGo(i90)) {
        /* V90 */
        if (canGo(i90.next(i90.getNodeDirection())))
          pushAndContinue(i90, edgeCost.getEdgeCostSlalom(FV90));
        /* 135 R */
        pushAndContinue(focus.next(d135), edgeCost.getEdgeCostSlalom(F135));
      }
    }
  }
}
bool StepMapSlalom::genPathFromMap(Indexes& path) const {
  path.clear();
  auto i = index_start.opposite();
  while (1) {
    path.push_back(i.opposite());
    /* ゴールなら終了 */
    if (cost_map[i.getIndex()] == 0)
      break;
    /* 移動元を確認 */
    const auto i_from = from_map[i.getIndex()];
    if (!i_from.getWallIndex().isInsideOfField())
      return false;
    /* コストが減っていなかったらおかしい */
    if (cost_map[i.getIndex()] <= cost_map[i_from.getIndex()])
      return false;
    i = from_map[i.getIndex()];
  }
  /* ゴールにたどり着いた */
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
  const auto get_min_cost_p = [&](int8_t x, int8_t y) {
    return std::min({
        cost_map[Index(x, y, 0, Direction::East).getIndex()],
        cost_map[Index(x, y, 0, Direction::North).getIndex()],
        cost_map[Index(x, y, 0, Direction::West).getIndex()],
        cost_map[Index(x, y, 0, Direction::South).getIndex()],
    });
  };
  const auto get_min_cost_w = [&](const WallIndex i) {
    return std::min({
        cost_map[Index(i.x, i.y, i.z, Direction::NorthEast).getIndex()],
        cost_map[Index(i.x, i.y, i.z, Direction::NorthWest).getIndex()],
        cost_map[Index(i.x, i.y, i.z, Direction::SouthWest).getIndex()],
        cost_map[Index(i.x, i.y, i.z, Direction::SouthEast).getIndex()],
    });
  };
  /* start to draw maze */
  for (int8_t y = MAZE_SIZE - 1; y >= -1; --y) {
    /* Horizontal Wall Line */
    for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
      os << "+";
      const auto w = maze.isWall(x, y, Direction::North);
      const auto k = maze.isKnown(x, y, Direction::North);
      const auto e = exists(Index(x, y, 1, Direction::NorthEast));
      /* Horizontal Wall */
      if (w)
        os << "---------";
      else
        os << "  " << (e ? C_YE : (k ? C_BL : C_RE)) << std::setw(5)
           << get_min_cost_w(WallIndex(x, y, 1)) << C_NO << "  ";
    }
    os << "+" << std::endl;
    /* Vertical Wall Line */
    if (y != -1) {
      os << "|  ";
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        /* Cell */
        if (exists(Index(x, y, 0, Direction::East)))
          os << C_YE << std::setw(5) << get_min_cost_p(x, y) << C_NO;
        else
          os << C_GR << std::setw(5) << get_min_cost_p(x, y) << C_NO;
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::East);
        const auto k = maze.isKnown(x, y, Direction::East);
        const auto e = exists(Index(x, y, 0, Direction::NorthEast));
        if (w)
          os << "  |  ";
        else
          os << (e ? C_YE : (k ? C_BL : C_RE)) << std::setw(5)
             << get_min_cost_w(WallIndex(x, y, 0)) << C_NO;
      }
      os << std::endl;
    }
  }
}
void StepMapSlalom::printPath(const Maze& maze,
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
  dest.reserve(src.size() * 4);
  for (const auto p : src)
    for (const auto nd : Direction::Along4)
      dest.push_back(Index(p, nd));
  return dest;
}
Directions StepMapSlalom::indexes2directions(const Indexes& path) {
  Directions dirs;
  dirs.reserve(path.size());
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
        case Direction::Back: /* F180 */
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
          MAZE_LOGE << "invalid Direction" << std::endl;
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
        case Direction::Left: /* V90 */
          dirs.push_back(nd + Direction::Left45);
          dirs.push_back(nd + Direction::Left135);
          break;
        case Direction::Right: /* V90 */
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
          MAZE_LOGE << "invalid Direction" << std::endl;
          break;
      }
    }
  }
  return dirs;
}

}  // namespace MazeLib
