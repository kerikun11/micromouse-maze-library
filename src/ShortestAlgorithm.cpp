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

#include <utility> /*< for std::index_sequence */

namespace MazeLib {

/**
 * @brief the cost implementation function
 *
 * @param i num of unit length straight [mm]
 * @param am accel max [mm/s/s]
 * @param vs velocity start [mm/s]
 * @param vm velocity max [mm/s]
 * @param isAlong true: along; false: diag
 * @return constexpr ShortestAlgorithm::cost_t time [ms]
 */
constexpr ShortestAlgorithm::cost_t gen_cost_impl(const int i, const float am,
                                                  const float vs,
                                                  const float vm,
                                                  const bool isAlong) {
  const auto d = (isAlong ? 90.0f : 45.0f * std::sqrt(2.0f)) *
                 (i + 1); /*< (i+1) 区画分の走行距離 */
  /* グラフの面積から時間を求める */
  const auto d_thr = (vm * vm - vs * vs) / am; /*< 最大速度に達する距離 */
  if (d < d_thr)
    return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 1000; /*< 三角加速 */
  else
    return (am * d + (vm - vs) * (vm - vs)) / (am * vm) * 1000; /*< 台形加速 */
}
template <std::size_t... vals>
constexpr auto gen_cost_table(std::index_sequence<vals...>, const float am,
                              const float vs, const float vm,
                              const bool isAlong) {
  return std::array<ShortestAlgorithm::cost_t, sizeof...(vals)>{
      {gen_cost_impl(vals, am, vs, vm, isAlong)...}};
}

ShortestAlgorithm::cost_t
ShortestAlgorithm::getEdgeCost(const enum ShortestAlgorithm::Pattern p,
                               const int n) {
  static constexpr auto am = 3000.0f; /*< 最大加速度 [mm/s/s] */
  static constexpr auto vs = 450.0f;  /*< 終始速度 [mm/s] */
  static constexpr auto vm = 2400.0f; /*< 飽和速度 [mm/s] */
  /* コストテーブルをコンパイル時生成 */
  static constexpr auto cost_table_along = gen_cost_table(
      std::make_index_sequence<MAZE_SIZE * 2>(), am, vs, vm, true);
  static constexpr auto cost_table_diag = gen_cost_table(
      std::make_index_sequence<MAZE_SIZE * 2>(), am, vs, vm, false);
  /* n along diag @ am = 3000, vs = 450, vm = 1800.
   * 1   158  118
   * 2   274  209
   * 3   370  286
   * 4   454  355
   */
  // for (int i = 0; i < MAZE_SIZE * 2; ++i) {
  //   std::cout << i + 1 << "\t" << cost_table_along[i] << "\t"
  //             << cost_table_diag[i] << std::endl;
  // }
  switch (p) {
  case ST_ALONG:
    return cost_table_along[n - 1]; /*< [ms] */
  case ST_DIAG:
    return cost_table_diag[n - 1]; /*< [ms] */
  case F45:
    return 249; /*< [ms] @ v = 425.272 [mm/s] */
  case F90:
    return 375; /*< [ms] @ v = 422.846 [mm/s] */
  case F135:
    return 421; /*< [ms] @ v = 375.888 [mm/s] */
  case F180:
    return 563; /*< [ms] @ v = 412.408 [mm/s] */
  case FV90:
    return 370; /*< [ms] @ v = 302.004 [mm/s] */
  case FS90:
    return 280; /*< [ms] @ v = 271.797 [mm/s] */
  }
  std::cerr << "Unknown Pattern" << std::endl;
  return 0;
}

/* union Index */

void ShortestAlgorithm::Index::uniquify(const Dir d) {
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
  case Dir::AbsMax:
    z = 0;
    break;
  default:
    loge << "invalid direction" << std::endl;
    break;
  }
}
const std::vector<
    std::pair<ShortestAlgorithm::Index, ShortestAlgorithm::cost_t>>
ShortestAlgorithm::Index::getSuccessors(const Maze &maze, const bool known_only,
                                        const bool diag_enabled) const {
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
      succs.push_back({Index(v_st, Dir::AbsMax, nd), getEdgeCost(ST_ALONG, n)});
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
            succs.push_back({Index(v_f, d_l, nd_45), getEdgeCost(F45)});
          if (canGo(v_fl, d_l)) //< 90度先の壁
            succs.push_back(
                {Index(v_fl, Dir::AbsMax, nd_90), getEdgeCost(F90)});
          const auto d_b = d_f + Dir::Back;    //< 後方向
          if (canGo(v_fl, d_b)) {              //< 135度の壁
            const auto v_fll = v_fl.next(d_b); //< 前左左の区画
            if (canGo(v_fll, d_l))             //< 135度行先
              succs.push_back({Index(v_fll, d_f, nd_135), getEdgeCost(F135)});
            if (canGo(v_fll, d_b)) //< 180度行先の壁
              succs.push_back(
                  {Index(v_fll, Dir::AbsMax, nd_180), getEdgeCost(F180)});
          }
        }
      }
    } else {
      /* 斜めなしのターン */
      const auto v_f = v.next(nd); //< i.e. vector front
      for (const auto d_turn : {Dir::Left, Dir::Right})
        if (canGo(v_f, nd + d_turn)) //< 90度方向の壁
          succs.push_back(
              {Index(v_f, Dir::AbsMax, nd + d_turn), getEdgeCost(FS90)});
    }
  } else {
    /* 壁の中央（斜めありの場合しかありえない） */
    /* 直前の壁 */
    const auto i_f = next(); //< i.e. index front
    if (!canGo(Vector(i_f), i_f.getDir())) {
      // loge << "FWE: " << *this << std::endl;
      return succs;
    }
    /* 直進で行けるところまで行く */
    auto i_st = i_f; //< i.e. index straight
    for (int8_t n = 1;; ++n) {
      auto i_ff = i_st.next(); //< 行先の壁
      if (!canGo(Vector(i_ff), i_ff.getDir()))
        break;
      succs.push_back({i_st, getEdgeCost(ST_DIAG, n)});
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
      succs.push_back({Index(v_45, Dir::AbsMax, d_45), getEdgeCost(F45)});
    /* V90方向, 135度方向*/
    if (canGo(v_45, d_135)) {
      /* V90方向, 135度方向*/
      auto v_135 = v_45.next(d_135);
      if (canGo(v_135, d_45))
        succs.push_back({Index(v_45, d_135, nd_90), getEdgeCost(FV90)});
      if (canGo(v_135, d_135))
        succs.push_back({Index(v_135, Dir::AbsMax, d_135), getEdgeCost(F135)});
    }
  }
  return succs;
}
const std::vector<
    std::pair<ShortestAlgorithm::Index, ShortestAlgorithm::cost_t>>
ShortestAlgorithm::Index::getPredecessors(const Maze &maze,
                                          const bool known_only,
                                          const bool diag_enabled) const {
  /* 斜めなしの predecessor は，単純な successor *
   * の逆にはならないので例外処理 */
  if (!diag_enabled) {
    /* 戻り値を用意 */
    std::vector<std::pair<Index, cost_t>> preds;
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
    /* 直進で行けるところまで行く */
    auto v_st = arrow_from(); //< i.e. vector straight
    for (int8_t n = 1;; ++n) {
      v_st = v_st.next(nd + Dir::Back);
      if (!canGo(v_st, nd))
        break;
      preds.push_back({Index(v_st, Dir::AbsMax, nd), getEdgeCost(ST_ALONG, n)});
    }
    /* ここからはターン */
    const auto v_b = arrow_from(); //< i.e. vector front
    /* 左右を一般化 */
    for (const auto d_turn : {Dir::Left, Dir::Right})
      if (canGo(v_b, nd + d_turn)) //< 90度方向の壁
        preds.push_back(
            {Index(v_b.next(nd + d_turn), Dir::AbsMax, nd + d_turn + Dir::Back),
             getEdgeCost(FS90)});
    return preds; /* 終了 */
  }
  /* それ以外 */
  auto preds = opposite().getSuccessors(maze, known_only, diag_enabled);
  for (auto &p : preds)
    p.first = p.first.opposite();
  return preds;
}
const ShortestAlgorithm::Index ShortestAlgorithm::Index::next() const {
  switch (nd) {
  /* 区画の中央 */
  case Dir::East:
  case Dir::North:
  case Dir::West:
  case Dir::South:
    return Index(Vector(x, y).next(nd), Dir::AbsMax, nd);
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
  loge << "invalid direction" << std::endl;
  return Index();
}

/* ShortestAlgorithm */

bool ShortestAlgorithm::calcShortestPath(Indexes &path, const bool known_only,
                                         const bool diag_enabled) {
  /* clear open_list */
  open_list.clear();
  std::make_heap(open_list.begin(), open_list.end(), greater);
  /* clear in_map */
  in_map.reset();
  /* clear f map */
  for (auto &f : f_map)
    f = CostMax;
  /* push the goal indexes */
  for (const auto v : maze.getGoals())
    for (const auto nd : Dir::ENWS()) {
      const auto i = Index(v, Dir::AbsMax, nd);
      f_map[i] = 0;
      in_map[i] = true;
      open_list.push_back(i);
      std::push_heap(open_list.begin(), open_list.end(), greater);
    }
  for (int j = 0;; max_iteration_size = std::max(max_iteration_size, ++j)) {
    // std::cout << "size():\t" << open_list.size() << std::endl;
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
    /* remove duplicated index */
    if (in_map[index] == false)
      continue;
    in_map[index] = false;
    /* successors */
    const auto succs = index.getSuccessors(maze, known_only, diag_enabled);
    for (const auto &s : succs) {
      if (!Vector(s.first).isInsideOfField())
        loge << "Out of Range! " << s.first << std::endl;
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
    const auto succs_opposite =
        index.opposite().getSuccessors(maze, known_only, diag_enabled);
    for (const auto &s : succs_opposite) {
      if (!Vector(s.first).isInsideOfField())
        loge << "Out of Range! " << s.first << std::endl;
      const auto f_p_new =
          f_map[index] - getHeuristic(index) + getHeuristic(s.first) + s.second;
      if (f_map[s.first] > f_p_new) {
        f_map[s.first] = f_p_new;
        from_map[s.first] = index.opposite();
        if (!in_map[s.first]) {
          open_list.push_back(s.first);
          std::push_heap(open_list.begin(), open_list.end(), greater);
        }
        in_map[s.first] = true;
      }
    }
  }
  /* post process to find the path*/
  path.clear();
  auto i = index_start.opposite();
  while (1) {
    path.push_back(i.opposite());
    if (f_map[i] == 0)
      break;
#if 1
    i = from_map[i];
#else
    /* find the index with the min cost */
    auto f_min = f_map[i];
    auto next = i;
    const auto predecessors = i.getPredecessors(maze, known_only, diag_enabled);
    for (const auto &p : predecessors) {
      if (!Vector(p.first).isInsideOfField())
        loge << "Out of Range! " << p.first << std::endl;
      const auto f_p = f_map[p.first] + p.second;
      if (f_min > f_p) {
        f_min = f_p;
        next = p.first;
      }
    }
    if (next == i) {
      logw << "No Path! " << i << std::endl;
      return false;
    }
    i = next;
#endif
  }
  return true;
}

void ShortestAlgorithm::printPath(std::ostream &os,
                                  const Indexes indexes) const {
  int steps[MAZE_SIZE][MAZE_SIZE] = {0};
  int counter = 1;
  for (const auto i : indexes) {
    auto v = Vector(i);
    steps[v.y][v.x] = counter++;
  }
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (int8_t x = 0; x < MAZE_SIZE; ++x) {
        if (steps[y][x] != 0)
          os << C_YELLOW << std::setw(3) << steps[y][x] << C_RESET;
        else
          os << "   ";
        os << (maze.isKnown(x, y, Dir::East)
                   ? (maze.isWall(x, y, Dir::East) ? "|" : " ")
                   : (C_RED "." C_RESET));
      }
      os << std::endl;
    }
    for (int8_t x = 0; x < MAZE_SIZE; ++x)
      os << "+"
         << (maze.isKnown(x, y, Dir::South)
                 ? (maze.isWall(x, y, Dir::South) ? "---" : "   ")
                 : (C_RED " . " C_RESET));
    os << "+" << std::endl;
  }
}

const Dirs ShortestAlgorithm::indexes2dirs(const Indexes &path,
                                           const bool diag_enabled) {
  if (!diag_enabled) {
    Dirs dirs;
    for (int i = 1; i < (int)path.size(); ++i) {
      const auto nd = path[i].getNodeDir();
      const auto v = Vector(path[i - 1]) - Vector(path[i]);
      for (int j = 0; j < std::abs(v.x) + std::abs(v.y); ++j)
        dirs.push_back(nd);
    }
    dirs.push_back(path.back().getNodeDir());
    return dirs;
  }
  Dirs dirs;
  for (int i = 0; i < (int)path.size() - 1; ++i) {
    const auto nd = path[i].getNodeDir();
    const auto rel_v = Vector(path[i + 1]) - Vector(path[i]);
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
