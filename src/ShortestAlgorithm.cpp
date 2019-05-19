#include "ShortestAlgorithm.h"

namespace MazeLib {

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
  default:
    std::cerr << __FILE__ << ":" << __LINE__ << " "
              << "invalid direction" << std::endl;
    break;
  }
}
void ShortestAlgorithm::Index::successors_for(
    const Maze &maze, const bool known_only, const bool diag_enabled,
    std::function<void(const Index, const cost_t)> callback) const {
  /* known_only を考慮した壁の判定式を用意 */
  auto canGo = [&](const Vector vec, const Dir dir) {
    /* スタートは袋小路なので例外処理 */
    if (vec == Vector(0, 0) && dir == Dir::South)
      return true;
    if (maze.isWall(vec, dir))
      return false;
    if (known_only && !maze.isKnown(vec, dir))
      return false;
    return true;
  };
  const auto v = Vector(x, y);
  /* 斜め禁止 */
  if (!diag_enabled) {
    /* 直前の壁 */
    if (!canGo(v, nd)) {
      /* ゴール区画だけあり得る */
      // std::cerr << "Something Wrong" << std::endl;
      return;
    }
    /* 直進で行けるところまで行く */
    int8_t n = 1;
    for (auto v_st = v.next(nd); canGo(v_st, nd); v_st = v_st.next(nd), ++n)
      callback(Index(v_st, Dir::AbsMax, nd), getEdgeCost(ST_ALONG, n));
    /* 左右ターン */
    const auto v_f = v.next(nd); //< i.e. vector front
    for (const auto d_turn : {Dir::Left, Dir::Right})
      if (canGo(v_f, nd + d_turn)) //< 90度方向の壁
        callback(Index(v_f, Dir::AbsMax, nd + d_turn), getEdgeCost(FS90));
    return;
  }
  /* 斜めあり */
  if (Dir(nd).isAlong()) {
    /* 区画の中央 */
    /* 直前の壁 */
    if (!canGo(v, nd)) {
      /* ゴール区画だけあり得る */
      // std::cerr << "Something Wrong" << std::endl;
      return;
    }
    /* 直進で行けるところまで行く */
    int8_t n = 1;
    for (auto v_st = v.next(nd); canGo(v_st, nd); v_st = v_st.next(nd), ++n)
      callback(Index(v_st, Dir::AbsMax, nd), getEdgeCost(ST_ALONG, n));
    /* ここからはターン */
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
          callback(Index(v_f, d_l, nd_45), getEdgeCost(F45));
        if (canGo(v_fl, d_l)) //< 90度先の壁
          callback(Index(v_fl, Dir::AbsMax, nd_90), getEdgeCost(F90));
        const auto d_b = d_f + Dir::Back;    //< 後方向
        if (canGo(v_fl, d_b)) {              //< 135度の壁
          const auto v_fll = v_fl.next(d_b); //< 前左左の区画
          if (canGo(v_fll, d_l))             //< 135度行先
            callback(Index(v_fll, d_f, nd_135), getEdgeCost(F135));
          if (canGo(v_fll, d_b)) //< 180度行先の壁
            callback(Index(v_fll, Dir::AbsMax, nd_180), getEdgeCost(F180));
        }
      }
    }
  } else {
    /* 壁の中央 */
    /* 直前の壁 */
    const auto i_f = this->next(); //< i.e. index front
    if (!canGo(Vector(i_f), i_f.getDir())) {
      // logw << "Something Wrong" << std::endl;
      return;
    }
    /* 直進で行けるところまで行く */
    auto i_st = i_f; //< i.e. index straight
    for (int8_t n = 1;; ++n) {
      auto i_ff = i_st.next(); //< 行先の壁
      if (!canGo(Vector(i_ff), i_ff.getDir()))
        break;
      callback(i_st, getEdgeCost(ST_DIAG, n));
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
      callback(Index(v_45, Dir::AbsMax, d_45), getEdgeCost(F45));
    /* V90方向, 135度方向*/
    if (canGo(v_45, d_135)) {
      /* V90方向, 135度方向*/
      auto v_135 = v_45.next(d_135);
      if (canGo(v_135, d_45))
        callback(Index(v_45, d_135, nd_90), getEdgeCost(FV90));
      if (canGo(v_135, d_135))
        callback(Index(v_135, Dir::AbsMax, d_135), getEdgeCost(F135));
    }
  }
}
void ShortestAlgorithm::Index::predecessors_for(
    const Maze &maze, const bool known_only, const bool diag_enabled,
    std::function<void(const Index, const cost_t)> callback) const {
  if (!diag_enabled) {
    /* known_only を考慮した壁の判定式を用意 */
    auto canGo = [&](const Vector vec, const Dir dir) {
      if (vec == Vector(0, 0) && dir == Dir::South)
        return true;
      if (maze.isWall(vec, dir))
        return false;
      if (known_only && !maze.isKnown(vec, dir))
        return false;
      return true;
    };
    /* 直進で行けるところまで行く */
    auto v_st = arrow_from(); //< 前方のマス
    for (int8_t n = 1;; n++) {
      if (!canGo(v_st, nd + Dir::Back))
        break;
      v_st = v_st.next(nd + Dir::Back);
      callback(Index(v_st, Dir::AbsMax, nd), getEdgeCost(ST_ALONG, n));
    }
    /* ここからはターン */
    const auto v_b = arrow_from(); //< i.e. vector front
    /* 左右を一般化 */
    for (const auto d_turn : {Dir::Left, Dir::Right})
      if (canGo(v_b, nd + d_turn)) //< 90度方向の壁
        callback(
            Index(v_b.next(nd + d_turn), Dir::AbsMax, nd + d_turn + Dir::Back),
            getEdgeCost(FS90));
    return;
  }
  opposite().successors_for(maze, known_only, diag_enabled,
                            [&callback](const Index i_n, const cost_t cost) {
                              callback(i_n.opposite(), cost);
                            });
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
    for (int8_t x = 0; x < MAZE_SIZE; x++)
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
