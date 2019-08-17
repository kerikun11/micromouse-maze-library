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

#include <cmath>   /*< for std::sqrt */
#include <utility> /*< for std::index_sequence */

namespace MazeLib {

/* union Index */

void Index::uniquify(const Dir d) {
  switch (d) {
  case Dir::East:
    z = 0;
    break;
  case Dir::North:
    z = 1;
    break;
  case Dir::West:
    z = 0;
    x--;
    break;
  case Dir::South:
    z = 1;
    y--;
    break;
  case Dir::Max:
    z = 0;
    break;
  default:
    loge << "invalid direction" << std::endl;
    break;
  }
}
const std::vector<std::pair<Index, cost_t>>
Index::getSuccessors(const Maze &maze, const EdgeCost &edge_cost,
                     const bool known_only, const bool diag_enabled) const {
  /* 戻り値を用意 */
  std::vector<std::pair<Index, cost_t>> succs;
  succs.reserve(MAZE_SIZE * 2);
  /* known_only を考慮した壁の判定式を用意 */
  const auto canGo = [&](const Vector vec, const Dir dir) {
    /* スタートは袋小路なので例外処理 */
    if (vec == Vector(0, 0) && dir == Dir::South)
      return true;
    if (maze.isWall(vec, dir))
      return false;
    if (known_only && !maze.isKnown(vec, dir))
      return false;
    return true;
  };
  const auto nd = getNodeDir();
  const auto v = Vector(x, y);
  if (nd.isAlong()) {
    /* 区画の中央 */
    /* 直前の壁 */
    if (!canGo(v, nd)) {
      /* ゴール区画だけあり得る */
      // loge << "FWE: " << *this << std::endl;
      return succs;
    }
    /* 直進で行けるところまで行く */
    int8_t n = 1;
    for (auto v_st = v.next(nd); canGo(v_st, nd); v_st = v_st.next(nd), ++n)
      succs.push_back(
          {Index(v_st, Dir::Max, nd), edge_cost.getEdgeCost(ST_ALONG, n)});
    if (diag_enabled) {
      /* 斜めありのターン */
      const auto d_f = nd;          //< i.e. dir front
      const auto v_f = v.next(d_f); //< i.e. vector front
      /* 左右を一般化 */
      for (const auto nd_rel_45 : {Dir::Left45, Dir::Right45}) {
        const auto nd_45 = nd + nd_rel_45;
        const auto nd_90 = nd + nd_rel_45 * 2;
        const auto nd_135 = nd + nd_rel_45 * 3;
        const auto nd_180 = nd + nd_rel_45 * 4;
        /* 横壁 */
        const auto d_l = nd_90;            //< 左方向
        if (canGo(v_f, d_l)) {             //< 45度方向の壁
          const auto v_fl = v_f.next(d_l); //< 前左の区画
          if (canGo(v_fl, d_f))            //< 45度先の壁
            succs.push_back(
                {Index(v_f, d_l, nd_45), edge_cost.getEdgeCost(F45)});
          if (canGo(v_fl, d_l)) //< 90度先の壁
            succs.push_back(
                {Index(v_fl, Dir::Max, nd_90), edge_cost.getEdgeCost(F90)});
          const auto d_b = d_f + Dir::Back;    //< 後方向
          if (canGo(v_fl, d_b)) {              //< 135度の壁
            const auto v_fll = v_fl.next(d_b); //< 前左左の区画
            if (canGo(v_fll, d_l))             //< 135度行先
              succs.push_back(
                  {Index(v_fll, d_f, nd_135), edge_cost.getEdgeCost(F135)});
            if (canGo(v_fll, d_b)) //< 180度行先の壁
              succs.push_back({Index(v_fll, Dir::Max, nd_180),
                               edge_cost.getEdgeCost(F180)});
          }
        }
      }
    } else {
      /* 斜めなしのターン */
      const auto v_f = v.next(nd); //< i.e. vector front
      for (const auto d_turn : {Dir::Left, Dir::Right})
        if (canGo(v_f, nd + d_turn)) //< 90度方向の壁
          succs.push_back(
              {Index(v_f, Dir::Max, nd + d_turn), edge_cost.getEdgeCost(FS90)});
    }
  } else {
    /* 壁の中央（斜めありの場合しかありえない） */
    /* 直前の壁 */
    const auto i_f = next(); //< i.e. index front
    if (!canGo(i_f.getVector(), i_f.getDir())) {
      // loge << "FWE: " << *this << std::endl;
      return succs;
    }
    /* 直進で行けるところまで行く */
    auto i_st = i_f; //< i.e. index straight
    for (int8_t n = 1;; ++n) {
      auto i_ff = i_st.next(); //< 行先の壁
      if (!canGo(i_ff.getVector(), i_ff.getDir()))
        break;
      succs.push_back({i_st, edge_cost.getEdgeCost(ST_DIAG, n)});
      i_st = i_ff;
    }
    /* ターン */
    auto nd_r45 = arrow_diag_to_along_45();
    auto d_45 = nd + nd_r45;
    auto nd_90 = nd + nd_r45 * 2;
    auto d_135 = nd + nd_r45 * 3;
    auto v_45 = i_f.arrow_to();
    /* 45度方向 */
    if (canGo(v_45, d_45))
      succs.push_back(
          {Index(v_45, Dir::Max, d_45), edge_cost.getEdgeCost(F45)});
    /* V90方向, 135度方向*/
    if (canGo(v_45, d_135)) {
      /* V90方向, 135度方向*/
      auto v_135 = v_45.next(d_135);
      if (canGo(v_135, d_45))
        succs.push_back(
            {Index(v_45, d_135, nd_90), edge_cost.getEdgeCost(FV90)});
      if (canGo(v_135, d_135))
        succs.push_back(
            {Index(v_135, Dir::Max, d_135), edge_cost.getEdgeCost(F135)});
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
    const auto canGo = [&](const Vector vec, const Dir dir) {
      // if (vec == Vector(0, 0) && dir == Dir::South)
      //   return true;
      if (maze.isWall(vec, dir))
        return false;
      if (known_only && !maze.isKnown(vec, dir))
        return false;
      return true;
    };
    const auto v_b = arrow_from();   //< i.e. vector back
    const auto d_b = nd + Dir::Back; //< i.e. dir back
    /* 直進で行けるところまで行く */
    int8_t n = 1;
    for (auto v_st = v_b.next(d_b); canGo(v_st, nd); v_st = v_st.next(d_b), ++n)
      preds.push_back(
          {Index(v_st, Dir::Max, nd), edge_cost.getEdgeCost(ST_ALONG, n)});
    /* ここからはターン */
    /* 左右を一般化 */
    for (const auto d_turn : {Dir::Left, Dir::Right})
      if (canGo(v_b, nd + d_turn)) //< 90度方向の壁
        preds.push_back(
            {Index(v_b.next(nd + d_turn), Dir::Max, nd + d_turn + Dir::Back),
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
const Index Index::next() const {
  switch (nd) {
  /* 区画の中央 */
  case Dir::East:
  case Dir::North:
  case Dir::West:
  case Dir::South:
    return Index(Vector(x, y).next(nd), Dir::Max, nd);
  /* 壁の中央 */
  case Dir::NorthEast:
    return z == 0 ? Index(Vector(x + 1, y), Dir::North, nd)
                  : Index(Vector(x, y + 1), Dir::East, nd);
  case Dir::NorthWest:
    return z == 0 ? Index(Vector(x, y), Dir::North, nd)
                  : Index(Vector(x - 1, y + 1), Dir::East, nd);
  case Dir::SouthWest:
    return z == 0 ? Index(Vector(x, y - 1), Dir::North, nd)
                  : Index(Vector(x - 1, y), Dir::East, nd);
  case Dir::SouthEast:
    return z == 0 ? Index(Vector(x + 1, y - 1), Dir::North, nd)
                  : Index(Vector(x, y), Dir::East, nd);
  default:
    break;
  }
  assert(1); /*< invalid direction */
  return Index();
}

/* ShortestAlgorithm */

bool ShortestAlgorithm::calcShortestPath(Indexes &path, const bool known_only,
                                         const bool diag_enabled) {
  /* min max */
  int8_t max_x = maze.getMaxX();
  int8_t max_y = maze.getMaxY();
  for (const auto v : maze.getGoals()) {
    max_x = std::max(v.x, max_x);
    max_y = std::max(v.y, max_y);
  }
  /* clear open_list */
  open_list.clear();
  /* clear in_map */
  in_map.reset();
  /* clear f map */
  for (auto &f : f_map)
    f = CostMax;
  /* push the goal indexes */
  for (const auto v : maze.getGoals())
    for (const auto nd : Dir::ENWS()) {
      const auto i = Index(v, Dir::Max, nd);
      f_map[i] = 0;
      in_map[i] = true;
      open_list.push_back(i);
      std::push_heap(open_list.begin(), open_list.end(), greater);
    }
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
      const auto v = s.first.getVector();
      if (v.isOutsideofField())
        loge << "Out of Range! " << s.first << std::endl;
      if (v.x > max_x + 1 || v.y > max_y + 1)
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
    i = from_map[i];
  }
  return true;
}

void ShortestAlgorithm::printPath(std::ostream &os,
                                  const Indexes indexes) const {
  int steps[MAZE_SIZE][MAZE_SIZE] = {0};
  int counter = 1;
  for (const auto i : indexes) {
    auto v = i.getVector();
    steps[v.y][v.x] = counter++;
  }
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (int8_t x = 0; x < MAZE_SIZE; ++x) {
        if (steps[y][x] != 0)
          os << C_YE << std::setw(3) << steps[y][x] << C_NO;
        else
          os << "   ";
        os << (maze.isKnown(x, y, Dir::East)
                   ? (maze.isWall(x, y, Dir::East) ? "|" : " ")
                   : (C_RE "." C_NO));
      }
      os << std::endl;
    }
    for (int8_t x = 0; x < MAZE_SIZE; ++x)
      os << "+"
         << (maze.isKnown(x, y, Dir::South)
                 ? (maze.isWall(x, y, Dir::South) ? "---" : "   ")
                 : (C_RE " . " C_NO));
    os << "+" << std::endl;
  }
}

const Dirs ShortestAlgorithm::indexes2dirs(const Indexes &path,
                                           const bool diag_enabled) {
  if (!diag_enabled) {
    Dirs dirs;
    for (int i = 1; i < (int)path.size(); ++i) {
      const auto nd = path[i].getNodeDir();
      const auto v = path[i - 1].getVector() - path[i].getVector();
      for (int j = 0; j < std::abs(v.x) + std::abs(v.y); ++j)
        dirs.push_back(nd);
    }
    return dirs;
  }
  Dirs dirs;
  for (int i = 0; i < (int)path.size() - 1; ++i) {
    const auto nd = path[i].getNodeDir();
    const auto rel_v = path[i + 1].getVector() - path[i].getVector();
    const auto rel_nd = Dir(path[i + 1].getNodeDir() - path[i].getNodeDir());
    if (nd.isAlong()) {
      switch (rel_nd) {
      case Dir::Front:
        for (int j = 0; j < std::abs(rel_v.x) + std::abs(rel_v.y); ++j)
          dirs.push_back(nd);
        break;
      case Dir::Left45:
        dirs.push_back(nd);
        dirs.push_back(nd + Dir::Left);
        break;
      case Dir::Right45:
        dirs.push_back(nd);
        dirs.push_back(nd + Dir::Right);
        break;
      case Dir::Left:
        dirs.push_back(nd);
        dirs.push_back(nd + Dir::Left);
        break;
      case Dir::Right:
        dirs.push_back(nd);
        dirs.push_back(nd + Dir::Right);
        break;
      case Dir::Left135:
        dirs.push_back(nd);
        dirs.push_back(nd + Dir::Left);
        dirs.push_back(nd + Dir::Back);
        break;
      case Dir::Right135:
        dirs.push_back(nd);
        dirs.push_back(nd + Dir::Right);
        dirs.push_back(nd + Dir::Back);
        break;
      case Dir::Back:
        dirs.push_back(nd);
        if (rel_v.rotate(-nd).y > 0) {
          dirs.push_back(nd + Dir::Left);
          dirs.push_back(nd + Dir::Back);
        } else {
          dirs.push_back(nd + Dir::Right);
          dirs.push_back(nd + Dir::Back);
        }
        break;
      }
    } else {
      switch (rel_nd) {
      case Dir::Front:
        for (auto index = path[i]; index != path[i + 1]; index = index.next()) {
          const auto nd_45 = index.arrow_diag_to_along_45();
          dirs.push_back(index.getNodeDir() + nd_45);
        }
        break;
      case Dir::Left45:
        dirs.push_back(nd + Dir::Left45);
        break;
      case Dir::Right45:
        dirs.push_back(nd + Dir::Right45);
        break;
      case Dir::Left:
        /* V90 */
        dirs.push_back(nd + Dir::Left45);
        dirs.push_back(nd + Dir::Left135);
        break;
      case Dir::Right:
        /* V90 */
        dirs.push_back(nd + Dir::Right45);
        dirs.push_back(nd + Dir::Right135);
        break;
      case Dir::Left135:
        dirs.push_back(nd + Dir::Left45);
        dirs.push_back(nd + Dir::Left135);
        break;
      case Dir::Right135:
        dirs.push_back(nd + Dir::Right45);
        dirs.push_back(nd + Dir::Right135);
        break;
      }
    }
  }
  return dirs;
}

} // namespace MazeLib
