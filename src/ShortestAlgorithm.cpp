#include "ShortestAlgorithm.h"

namespace MazeLib {

/**
 * @brief 台形加速にかかる時間を算出する関数
 *
 * @param am 最大加速度の大きさ [m/s/s]
 * @param vs 初速および最終速度の大きさ [m/s]
 * @param vm 飽和速度の大きさ [m/s]
 * @param d 走行距離 [m]
 * @return float 走行時間 [s]
 */
static float calcStraightTime(const float am, const float vs, const float vm,
                              const float d) {
  /* グラフの面積から時間を求める */
  const auto d_thr =
      (vm * vm - vs * vs) / am; /*< 最大速度にちょうど達する距離 */
  if (d < d_thr)
    return 2 * (std::sqrt(vs * vs + am * d) - vs) / am; /*< 三角加速 */
  else
    return (am * d + (vm - vs) * (vm - vs)) / (am * vm); /*< 台形加速 */
}

ShortestAlgorithm::cost_t
ShortestAlgorithm::getEdgeCost(const enum ShortestAlgorithm::Pattern p,
                               const int n) {
  static std::array<cost_t, MAZE_SIZE * 2> cost_table_along;
  static std::array<cost_t, MAZE_SIZE * 2> cost_table_diag;
  static bool initialized = false; /*< 初回のみ実行するように設定 */
  if (!initialized) {
    initialized = true;
    /* 台形加速のコストテーブルを事前に用意 */
    static const float am = 3000.0f; /*< 最大加速度 [mm/s/s] */
    static const float vs = 450.0f;  /*< 終始速度 [mm/s] */
    static const float vm = 2400.0f; /*< 飽和速度 [mm/s] */
    for (int i = 0; i < MAZE_SIZE * 2; ++i) {
      const float d_along = 90.0f * (i + 1); /*< 走行距離 [mm] */
      const float d_diag = 1.41421356f * 45.0f * (i + 1); /*< 走行距離 [mm] */
      cost_table_along[i] =
          calcStraightTime(am, vs, vm, d_along) * 1000; /*< [ms] */
      cost_table_diag[i] =
          calcStraightTime(am, vs, vm, d_diag) * 1000; /*< [ms] */
      // std::cout << i + 1 << "\t" << cost_table_along[i] << "\t"
      //           << cost_table_diag[i] << std::endl;
    }
    /* n along diag @ am = 3000, vs = 450, vm = 1800.
     * 1  158   118
     * 2  274   209
     * 3  370   286
     * 4  454   355
     */
  }
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

/* Index */

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
    std::cerr << __FILE__ << ":" << __LINE__ << " "
              << "invalid direction" << std::endl;
    break;
  }
}
const std::vector<
    std::pair<ShortestAlgorithm::Index, ShortestAlgorithm::cost_t>>
ShortestAlgorithm::Index::getSuccessors(const Maze &maze, const bool known_only,
                                        const bool diag_enabled) const {
  /* 戻り値を用意 */
  std::vector<std::pair<Index, cost_t>> succs;
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
      // std::cerr << __FILE__ << ":" << __LINE__ << " "
      //           << "FWE: " << *this << std::endl;
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
      // std::cerr << __FILE__ << ":" << __LINE__ << " "
      //           << "FWE: " << *this << std::endl;
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

/* ShortestAlgorithm */

bool ShortestAlgorithm::calcShortestPath(Indexes &path, const bool known_only,
                                         const bool diag_enabled) {
  /* clear open_list */
  open_list.clear();
  std::make_heap(open_list.begin(), open_list.end(), greater);
  /* clear node map */
  for (auto &node : f_map)
    node = CostMax;
  /* push the goal indexes */
  for (const auto v : maze.getGoals())
    for (const auto nd : Dir::ENWS()) {
      const auto i = Index(v, Dir::AbsMax, nd);
      f_map[i] = 0;
      open_list.push_back(i);
      std::push_heap(open_list.begin(), open_list.end(), greater);
    }
  while (1) {
    // std::cout << "size():\t" << open_list.size() << std::endl;
    if (open_list.empty()) {
      std::cerr << __FILE__ << ":" << __LINE__ << " "
                << "open_list is empty " << std::endl;
      return false;
    }
    /* place the element with the min cost to back */
    std::pop_heap(open_list.begin(), open_list.end(), greater);
    const auto index = open_list.back();
    open_list.pop_back();
    /* breaking condition */
    if (index == index_start)
      break;
    const auto succs = index.getSuccessors(maze, known_only, diag_enabled);
    for (const auto &s : succs) {
      if (!Vector(s.first).isInsideOfField())
        std::cerr << __FILE__ << ":" << __LINE__ << " "
                  << "Warning! " << s.first << std::endl;
      if (f_map[s.first] > f_map[index] + s.second) {
        f_map[s.first] = f_map[index] + s.second;
        open_list.push_back(s.first);
        std::push_heap(open_list.begin(), open_list.end(), greater);
      }
    }
  }
  /* post process */
  path.erase(path.begin(), path.end());
  auto i = index_start;
  while (1) {
    path.push_back(i.opposite());
    if (f_map[i] == 0)
      break;
    /* find the index with the min cost */
    auto min_cost = f_map[i];
    auto next = i;
    const auto preds = i.getPredecessors(maze, known_only, diag_enabled);
    for (const auto p : preds) {
      if (!Vector(p.first).isInsideOfField())
        std::cerr << __FILE__ << ":" << __LINE__ << " "
                  << "Warning! " << p.first << std::endl;
      const auto cost_p = f_map[p.first];
      if (cost_p < min_cost) {
        min_cost = cost_p;
        next = p.first;
      }
    }
    if (next == i) {
      std::cerr << __FILE__ << ":" << __LINE__ << " "
                << "No Path! " << i << std::endl;
      while (1)
        ;
      return false;
    }
    i = next;
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
