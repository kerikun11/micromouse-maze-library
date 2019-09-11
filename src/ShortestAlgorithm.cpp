/**
 * @file ShortestAlgorithm.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief Shortest Algorithm for Micromouse
 * @version 0.1
 * @date 2019-05-26
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "ShortestAlgorithm.h"

#include <cmath> /*< for std::sqrt */

namespace MazeLib {

/* union Index */
void Index::uniquify(const Direction d) {
  switch (d) {
  case Direction::East:
    z = 0;
    break;
  case Direction::North:
    z = 1;
    break;
  case Direction::West:
    z = 0;
    x--;
    break;
  case Direction::South:
    z = 1;
    y--;
    break;
  case Direction::Max:
    z = 0;
    break;
  default:
    loge << "invalid direction" << std::endl;
    break;
  }
}
const Position Index::arrow_from() const {
  switch (nd) {
  case Direction::East:
  case Direction::North:
  case Direction::West:
  case Direction::South:
    return Position(x, y);
  case Direction::NorthEast:
    return Position(x, y);
  case Direction::NorthWest:
    return z == 0 ? Position(x + 1, y) : Position(x, y);
  case Direction::SouthWest:
    return z == 0 ? Position(x + 1, y) : Position(x, y + 1);
  case Direction::SouthEast:
    return z == 0 ? Position(x, y) : Position(x, y + 1);
  default:
    break;
  }
  logw << "Invalid Direction: " << nd << std::endl;
  return Position(x, y);
}
const Position Index::arrow_to() const {
  switch (nd) {
  case Direction::East:
  case Direction::North:
  case Direction::West:
  case Direction::South:
    return Position(x, y).next(nd);
  case Direction::NorthEast:
    return z == 0 ? Position(x + 1, y) : Position(x, y + 1);
  case Direction::NorthWest:
    return z == 0 ? Position(x, y) : Position(x, y + 1);
  case Direction::SouthWest:
    return Position(x, y);
  case Direction::SouthEast:
    return z == 0 ? Position(x + 1, y) : Position(x, y);
  default:
    break;
  }
  logw << "Invalid Direction: " << nd << std::endl;
  return Position(x, y);
}
const Direction Index::arrow_diag_to_along_45() const {
  switch (nd) {
  case Direction::NorthEast:
  case Direction::SouthWest:
    return z == 0 ? Direction::Left45 : Direction::Right45;
  case Direction::NorthWest:
  case Direction::SouthEast:
    return z == 1 ? Direction::Left45 : Direction::Right45;
  default:
    logw << "Invalid Direction: " << nd << std::endl;
    return Direction::Max;
  }
}
const Index Index::next() const {
  switch (getNodeDirection()) {
  case Direction::East:
  case Direction::North:
  case Direction::West:
  case Direction::South:
    return getNodeDirection().isAlong()
               ? Index(getPosition().next(nd), nd)
               : Index(WallIndex(Position(x, y), getNodeDirection()).next(nd),
                       nd);
  case Direction::NorthEast:
  case Direction::NorthWest:
  case Direction::SouthWest:
  case Direction::SouthEast:
    return Index(WallIndex(x, y, z).next(nd), nd);
  default:
    logw << "Invalid Direction: " << nd << std::endl;
    return Index(x, y, z, nd);
  }
}
const std::vector<std::pair<Index, cost_t>>
Index::getSuccessors(const Maze &maze, const EdgeCost &edge_cost,
                     const bool known_only, const bool diag_enabled) const {
  /* 戻り値を用意 */
  std::vector<std::pair<Index, cost_t>> succs;
  succs.reserve(MAZE_SIZE * 2);
  /* known_only を考慮した壁の判定式を用意 */
  const auto canGo = [&](const Position vec, const Direction dir) {
    /* スタートは袋小路なので例外処理 */
    if (vec == Position(0, 0) && dir == Direction::South)
      return true;
    if (maze.isWall(vec, dir))
      return false;
    if (known_only && !maze.isKnown(vec, dir))
      return false;
    return true;
  };
  const auto nd = getNodeDirection();
  const auto p = Position(x, y);
  if (nd.isAlong()) {
    /* 区画の中央 */
    /* 直前の壁 */
    if (!canGo(p, nd)) {
      /* ゴール区画だけあり得る */
      // loge << "FWE: " << *this << std::endl;
      return succs;
    }
    /* 直進で行けるところまで行く */
    int8_t n = 1;
    for (auto p_st = p.next(nd); canGo(p_st, nd); p_st = p_st.next(nd), ++n)
      succs.push_back({Index(p_st, nd), edge_cost.getEdgeCost(ST_ALONG, n)});
    if (diag_enabled) {
      /* 斜めありのターン */
      const auto d_f = nd;          //< i.e. dir front
      const auto p_f = p.next(d_f); //< i.e. vector front
      /* 左右を一般化 */
      for (const auto nd_rel_45 : {Direction::Left45, Direction::Right45}) {
        const auto nd_45 = nd + nd_rel_45;
        const auto nd_90 = nd + nd_rel_45 * 2;
        const auto nd_135 = nd + nd_rel_45 * 3;
        const auto nd_180 = nd + nd_rel_45 * 4;
        /* 横壁 */
        const Direction d_l = nd_90;       //< 左方向
        if (canGo(p_f, d_l)) {             //< 45度方向の壁
          const auto p_fl = p_f.next(d_l); //< 前左の区画
          if (canGo(p_fl, d_f))            //< 45度先の壁
            succs.push_back(
                {Index(p_f, d_l, nd_45), edge_cost.getEdgeCost(F45)});
          if (canGo(p_fl, d_l)) //< 90度先の壁
            succs.push_back({Index(p_fl, nd_90), edge_cost.getEdgeCost(F90)});
          const auto d_b = d_f + Direction::Back; //< 後方向
          if (canGo(p_fl, d_b)) {                 //< 135度の壁
            const auto p_fll = p_fl.next(d_b);    //< 前左左の区画
            if (canGo(p_fll, d_l))                //< 135度行先
              succs.push_back(
                  {Index(p_fll, d_f, nd_135), edge_cost.getEdgeCost(F135)});
            if (canGo(p_fll, d_b)) //< 180度行先の壁
              succs.push_back(
                  {Index(p_fll, nd_180), edge_cost.getEdgeCost(F180)});
          }
        }
      }
    } else {
      /* 斜めなしのターン */
      const auto p_f = p.next(nd); //< i.e. vector front
      for (const auto d_turn : {Direction::Left, Direction::Right})
        if (canGo(p_f, nd + d_turn)) //< 90度方向の壁
          succs.push_back(
              {Index(p_f, nd + d_turn), edge_cost.getEdgeCost(FS90)});
    }
  } else {
    /* 壁の中央（斜めありの場合しかありえない） */
    /* 直前の壁 */
    const auto i_f = next(); //< i.e. index front
    if (!canGo(i_f.getPosition(), i_f.getDirection())) {
      // loge << "FWE: " << *this << std::endl;
      return succs;
    }
    /* 直進で行けるところまで行く */
    auto i_st = i_f; //< i.e. index straight
    for (int8_t n = 1;; ++n) {
      auto i_ff = i_st.next(); //< 行先の壁
      if (!canGo(i_ff.getPosition(), i_ff.getDirection()))
        break;
      succs.push_back({i_st, edge_cost.getEdgeCost(ST_DIAG, n)});
      i_st = i_ff;
    }
    /* ターン */
    const auto nd_r45 = arrow_diag_to_along_45();
    const auto d_45 = nd + nd_r45;
    const auto nd_90 = nd + nd_r45 * 2;
    const Direction d_135 = nd + nd_r45 * 3;
    const auto p_45 = i_f.arrow_to();
    /* 45度方向 */
    if (canGo(p_45, d_45))
      succs.push_back({Index(p_45, d_45), edge_cost.getEdgeCost(F45)});
    /* V90方向, 135度方向*/
    if (canGo(p_45, d_135)) {
      /* V90方向, 135度方向*/
      const auto p_135 = p_45.next(d_135);
      if (canGo(p_135, d_45))
        succs.push_back(
            {Index(p_45, d_135, nd_90), edge_cost.getEdgeCost(FV90)});
      if (canGo(p_135, d_135))
        succs.push_back({Index(p_135, d_135), edge_cost.getEdgeCost(F135)});
    }
  }
  return succs;
}
const std::vector<std::pair<Index, cost_t>>
Index::getPredecessors(const Maze &maze, const EdgeCost &edge_cost,
                       const bool known_only, const bool diag_enabled) const {
  /* 斜めなしの predecessor は，単純な successor *
   * の逆にはならないので例外処理 */
  if (!diag_enabled) {
    /* 戻り値を用意 */
    std::vector<std::pair<Index, cost_t>> preds;
    preds.reserve(MAZE_SIZE * 2);
    /* known_only を考慮した壁の判定式を用意 */
    const auto canGo = [&](const Position vec, const Direction dir) {
      // if (vec == Position(0, 0) && dir == Direction::South)
      //   return true;
      if (maze.isWall(vec, dir))
        return false;
      if (known_only && !maze.isKnown(vec, dir))
        return false;
      return true;
    };
    const auto p_b = arrow_from();         //< i.e. vector back
    const auto d_b = nd + Direction::Back; //< i.e. dir back
    /* 直進で行けるところまで行く */
    int8_t n = 1;
    for (auto p_st = p_b.next(d_b); canGo(p_st, nd); p_st = p_st.next(d_b), ++n)
      preds.push_back({Index(p_st, nd), edge_cost.getEdgeCost(ST_ALONG, n)});
    /* ここからはターン */
    /* 左右を一般化 */
    for (const auto d_turn : {Direction::Left, Direction::Right})
      if (canGo(p_b, nd + d_turn)) //< 90度方向の壁
        preds.push_back(
            {Index(p_b.next(nd + d_turn), nd + d_turn + Direction::Back),
             edge_cost.getEdgeCost(FS90)});
    return preds; /* 終了 */
  }
  /* それ以外 */
  auto preds =
      opposite().getSuccessors(maze, edge_cost, known_only, diag_enabled);
  for (auto &p : preds)
    p.first = p.first.opposite();
  return preds;
}

/* ShortestAlgorithm */
bool ShortestAlgorithm::calcShortestPath(Indexes &path, const bool known_only,
                                         const bool diag_enabled) {
  /* min max */
  int8_t max_x = maze.getMaxX();
  int8_t max_y = maze.getMaxY();
  for (const auto p : maze.getGoals()) {
    max_x = std::max(p.x, max_x);
    max_y = std::max(p.y, max_y);
  }
  /* clear open_list */
  open_list.clear();
  /* clear in_map */
  in_map.reset();
  /* clear f map */
  for (auto &f : f_map)
    f = CostMax;
  /* push the goal indexes */
  for (const auto p : maze.getGoals())
    for (const auto nd : Direction::getAlong4()) {
      const auto i = Index(p, nd);
      f_map[i] = 0;
      in_map[i] = true;
      open_list.push_back(i);
      std::push_heap(open_list.begin(), open_list.end(), greater);
    }
  /* iteration */
  for (int j = 0;; max_iteration_size = std::max(max_iteration_size, ++j)) {
    if (max_open_list_size < (int)open_list.size())
      max_open_list_size = open_list.size();
    if (open_list.empty()) {
      logw << "open_list is empty! " << std::endl;
      return false;
    }
    /* place the element with the min cost to back */
    // std::make_heap(open_list.begin(), open_list.end(), greater);
    std::pop_heap(open_list.begin(), open_list.end(), greater);
    const auto index = open_list.back();
    open_list.pop_back();
    // logi << index << std::endl;
    /* breaking condition */
    if (index == index_start.opposite())
      break;
    /* ignore duplicated index */
    if (in_map[index] == false)
      continue;
    in_map[index] = false;
    /* successors */
    const auto succs =
        index.getSuccessors(maze, edge_cost, known_only, diag_enabled);
    for (const auto &s : succs) {
      const auto p = s.first.getPosition();
      if (!p.isInsideOfField())
        loge << "Out of Range! " << s.first << std::endl;
      if (p.x > max_x + 1 || p.y > max_y + 1)
        continue;
      const auto f_p_new =
          f_map[index] - getHeuristic(index) + getHeuristic(s.first) + s.second;
      if (f_map[s.first] > f_p_new) {
        f_map[s.first] = f_p_new;
        from_map[s.first] = index;
        if (!in_map[s.first]) {
          open_list.push_back(s.first);
          std::push_heap(open_list.begin(), open_list.end(), greater);
        }
        in_map[s.first] = true;
      }
    }
  }
  /* post process to find the path */
  path.clear();
  auto i = index_start.opposite();
  while (1) {
    path.push_back(i.opposite());
    if (f_map[i] == 0)
      break;
    if (f_map[i] <= f_map[from_map[i]])
      return false;
    i = from_map[i];
  }
  return true;
}
void ShortestAlgorithm::print(const Indexes indexes, std::ostream &os) const {
  int steps[MAZE_SIZE][MAZE_SIZE] = {{0}};
  int counter = 1;
  for (const auto i : indexes) {
    auto p = i.getPosition();
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
const Directions ShortestAlgorithm::indexes2dirs(const Indexes &path,
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
        logw << "invalid direction" << std::endl;
        break;
      }
    } else {
      switch (rel_nd) {
      case Direction::Front:
        for (auto index = path[i]; index != path[i + 1]; index = index.next()) {
          const auto nd_45 = index.arrow_diag_to_along_45();
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
        logw << "invalid direction" << std::endl;
        break;
      }
    }
  }
  return dirs;
}

} // namespace MazeLib
