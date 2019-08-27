/**
 * @file ShortestAlgorithm.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief Shortest Algorithm for Micromouse
 * @version 0.1
 * @date 2019-05-11
 *
 * @copyright Copyright (c) 2019
 *
 */
#pragma once

#include "Maze.h"

#include <algorithm> /*< for std::find_if, etc. */
#include <bitset>
#include <functional>
#include <iomanip> /*< for std::setw() */
#include <limits>  /*< for std::numeric_limits */

#include <cmath>
#include <queue>

namespace MazeLib {

using cost_t = uint16_t; /**< @brief 時間コストの型 [ms] */
static constexpr cost_t CostMax = std::numeric_limits<cost_t>::max();
/**
 * @brief 最短走行パターン
 */
enum Pattern : int8_t { ST_ALONG, ST_DIAG, F45, F90, F135, F180, FV90, FS90 };

struct Action {
  enum Type {
    Straight,
    Slalom,
    TypeMax,
  };
  enum Slalom {
    F45,
    F90,
    F135,
    F180,
    FV90,
    FS90,
    SlalomMax,
  };
  enum Dir {
    Left,
    Right,
  };
  Type type;
  int index;
  Dir dir;
};

struct RunParameter {
  float slalom_gain[Action::SlalomMax];
  float v_max;
  float a_max;
};

class EdgeCost {
public:
  struct RunParameter {
    RunParameter() {}
    float vs = 450.0f;    /*< 基本速度 [mm/s] */
    float am_a = 4800.0f; /*< 最大加速度 [mm/s/s] */
    float am_d = 3600.0f; /*< 最大加速度(斜め) [mm/s/s] */
    float vm_a = 1800.0f; /*< 飽和速度 [mm/s] */
    float vm_d = 1200.0f; /*< 飽和速度(斜め) [mm/s] */
    cost_t t_F45 = 249;   /*< [ms] @ v = 425.272 [mm/s] */
    cost_t t_F90 = 375;   /*< [ms] @ v = 422.846 [mm/s] */
    cost_t t_F135 = 421;  /*< [ms] @ v = 375.888 [mm/s] */
    cost_t t_F180 = 563;  /*< [ms] @ v = 412.408 [mm/s] */
    cost_t t_FV90 = 370;  /*< [ms] @ v = 302.004 [mm/s] */
    cost_t t_FS90 = 280;  /*< [ms] @ v = 271.797 [mm/s] */
  };

public:
  EdgeCost(const struct RunParameter rp = RunParameter()) : rp(rp) {
    genCostTable();
  }
  cost_t getEdgeCost(const Pattern p, const int n = 1) const {
    switch (p) {
    case ST_ALONG:
      return cost_table_along[n - 1]; /*< [ms] */
    case ST_DIAG:
      return cost_table_diag[n - 1]; /*< [ms] */
    case F45:
      return rp.t_F45; /*< [ms] */
    case F90:
      return rp.t_F90; /*< [ms] */
    case F135:
      return rp.t_F135; /*< [ms] */
    case F180:
      return rp.t_F180; /*< [ms] */
    case FV90:
      return rp.t_FV90; /*< [ms] */
    case FS90:
      return rp.t_FS90; /*< [ms] */
    }
    std::cerr << "Unknown Pattern" << std::endl;
    return 0;
  }
  const RunParameter &getRunParameter() const { return rp; }
  void setRunParameter(const RunParameter &rp) {
    this->rp = rp;
    genCostTable();
  }

private:
  RunParameter rp;
  std::array<cost_t, MAZE_SIZE * 2> cost_table_along;
  std::array<cost_t, MAZE_SIZE * 2> cost_table_diag;
  cost_t gen_cost_impl(const int i, const float am, const float vs,
                       const float vm, const float seg) {
    const auto d = seg * (i + 1); /*< (i+1) 区画分の走行距離 */
    /* グラフの面積から時間を求める */
    const auto d_thr = (vm * vm - vs * vs) / am; /*< 最大速度に達する距離 */
    if (d < d_thr)
      return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 1000; /*< 三角加速 */
    else
      return (am * d + (vm - vs) * (vm - vs)) / (am * vm) *
             1000; /*< 台形加速 */
  }
  void genCostTable() {
    const float seg_a = 90.0f;
    const float seg_d = 45.0f * std::sqrt(2);
    for (int i = 0; i < MAZE_SIZE * 2; ++i) {
      cost_table_along[i] = gen_cost_impl(i, rp.am_a, rp.vs, rp.vm_a, seg_a);
      cost_table_diag[i] = gen_cost_impl(i, rp.am_d, rp.vs, rp.vm_d, seg_d);
      // std::cout << i + 1 << "\t" << cost_table_along[i] << "\t"
      //           << cost_table_diag[i] << std::endl;
    }
  }
};

/**
 * @brief Graph の Node の Index．
 * 「各区画中央の4方位」または「 各壁上の4方位」，の位置姿勢を一意に識別する
 */
union __attribute__((__packed__)) Index {
private:
  struct __attribute__((__packed__)) {
    int x : 6; /**< @brief x coordinate of the cell */
    int y : 6; /**< @brief y coordinate of the cell */
    unsigned int
        z : 1; /**< @brief position assignment in the cell, 0:East; 1:North */
    unsigned int nd : 3; /**< @brief direction of the node */
  };
  unsigned int all : 16; /**< @brief union element for all access */

public:
#define INDEX_ARRANGEMENT 3
#if INDEX_ARRANGEMENT == 0
  static constexpr int Max = MAZE_SIZE * MAZE_SIZE * 8;
#elif INDEX_ARRANGEMENT == 1
  static constexpr int Max = MAZE_SIZE * MAZE_SIZE * 16;
#elif INDEX_ARRANGEMENT == 2
  static constexpr int Max = MAZE_SIZE * MAZE_SIZE * 6;
#elif INDEX_ARRANGEMENT == 3
  static constexpr int Max = MAZE_SIZE * MAZE_SIZE * 12;
#endif

public:
  /**
   * @brief Construct a new Index object
   */
  Index(const int8_t x, const int8_t y, const Dir d, const Dir nd)
      : x(x), y(y), nd(nd) {
    uniquify(d);
  }
  Index(const Vector v, const Dir d, const Dir nd) : x(v.x), y(v.y), nd(nd) {
    uniquify(d);
  }
  Index() : all(0) {}
  /**
   * @brief unique な ID を返す
   */
  operator uint16_t() const {
#if INDEX_ARRANGEMENT == 0
    return ((nd & 3) << 11) | (z << 10) | (y << 5) | x; /*< M * M * 8 */
#elif INDEX_ARRANGEMENT == 1
    return (nd << 11) | (z << 10) | (y << 5) | x; /*< M * M * 16 */
#elif INDEX_ARRANGEMENT == 2
    return (((~nd) & 1) << 12) | (z << 11) | (nd & 2) << 9 | (y << 5) |
           x; /*< M * M * 6 */
#elif INDEX_ARRANGEMENT == 3
    return (((~nd) & 1) << (2 * MAZE_SIZE_BIT + 3)) |
           (z << (2 * MAZE_SIZE_BIT + 2)) |
           ((6 & nd) << (2 * MAZE_SIZE_BIT - 1)) | (x << MAZE_SIZE_BIT) |
           y; /*< M * M * 12 */
#endif
  }
  /**
   * @brief 座標の冗長を一意にする．
   * d を East or North のどちらかにそろえる
   */
  void uniquify(const Dir d);
  /**
   * @brief Getters
   */
  const Dir getDir() const { return z == 0 ? Dir::East : Dir::North; }
  const Dir getNodeDir() const { return nd; }
  const Vector getVector() const { return Vector(x, y); }
  friend std::ostream &operator<<(std::ostream &os, const Index i) {
    return os << "( " << std::setw(2) << (int)i.x << ", " << std::setw(2)
              << (int)i.y << ", " << i.getDir().toChar() << ", "
              << i.getNodeDir().toChar() << ")";
  }
  const Vector arrow_from() const;
  const Vector arrow_to() const;
  /**
   * @brief 斜め方向に向いているときの区画への相対方向(±45度)を返す
   * @return const Dir Dir::Left45 or Dir::Right45
   */
  const Dir arrow_diag_to_along_45() const;
  /**
   * @brief NodeDir が向いている方向の隣の Index を返す
   * @return const Index
   */
  const Index next() const;
  const Index opposite() const { return Index(x, y, getDir(), nd + Dir::Back); }
  const std::vector<std::pair<Index, cost_t>>
  getSuccessors(const Maze &maze, const EdgeCost &edge_cost,
                const bool known_only, const bool diag_enabled) const;
  const std::vector<std::pair<Index, cost_t>>
  getPredecessors(const Maze &maze, const EdgeCost &edge_cost,
                  const bool known_only, const bool diag_enabled) const;
};
static_assert(sizeof(Index) == 2, "size error"); /**< size check */
using Indexes = std::vector<Index>;

/**
 * @brief 最短経路導出アルゴリズム
 */
class ShortestAlgorithm {
public:
  ShortestAlgorithm(const Maze &maze)
      : maze(maze), greater([&](const auto i1, const auto i2) {
          return f_map[i1] > f_map[i2];
        }) {}

public:
  /**
   * @brief Get the Heuristic Value
   * @param i Index
   * @return cost_t heuristic value
   */
  cost_t getHeuristic(const Index i) const {
    return getHeuristic(i, index_start);
  }
  cost_t getHeuristic(const Index i, const Index s) const {
    // return 0;
    const auto v = i.getVector() - s.getVector();
    // const auto d = std::sqrt(v.x * v.x + v.y * v.y);
    const auto d = std::max(std::abs(v.x), std::abs(v.y));
    return edge_cost.getEdgeCost(ST_DIAG, d);
  }
  /**
   * @brief 最短経路を求める
   *
   * @param path 結果を入れる箱
   * @param known_only
   * @return true 成功
   * @return false 失敗
   */
  bool calcShortestPath(Indexes &path, const bool known_only,
                        const bool diag_enabled);
  /**
   * @brief コストマップの更新
   */
  void update(const Maze &maze, const EdgeCost &edge_cost, const Indexes &dest,
              const bool known_only, const bool diag_enabled) {
    /* 全ノードのコストを最大値に設定 */
    for (auto &f : f_map)
      f = CostMax;
    /* 更新予約のキュー */
    std::queue<Index> q;
    /* dest のコストを0とする */
    for (const auto i : dest) {
      f_map[i] = 0;
      q.push(i);
    }
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
    /* 更新がなくなるまで更新 */
    while (!q.empty()) {
      const auto focus = q.front();
      q.pop();
      const auto focus_cost = f_map[focus];
      const auto pushAndContinue = [&](const Index next, const cost_t cost) {
        const auto next_cost = focus_cost + cost;
        if (f_map[next] <= next_cost)
          return false;
        f_map[next] = next_cost;
        from_map[next] = focus;
        q.push(next);
        return true;
      };
      const auto nd = focus.getNodeDir();
      const auto v = focus.getVector();
      if (nd.isAlong()) {
        /* 区画の中央 */
        /* 直前の壁 */
        if (!canGo(v, nd)) {
          /* ゴール区画だけあり得る */
          // loge << "FWE: " << focus << std::endl;
          continue;
        }
        /* 直進で行けるところまで行く */
        int8_t n = 1;
        for (auto v_st = v.next(nd); canGo(v_st, nd); v_st = v_st.next(nd), ++n)
          if (!pushAndContinue(Index(v_st, Dir::Max, nd),
                               edge_cost.getEdgeCost(ST_ALONG, n)))
            break;
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
                pushAndContinue(Index(v_f, d_l, nd_45),
                                edge_cost.getEdgeCost(F45));
              if (canGo(v_fl, d_l)) //< 90度先の壁
                pushAndContinue(Index(v_fl, Dir::Max, nd_90),
                                edge_cost.getEdgeCost(F90));
              const auto d_b = d_f + Dir::Back;    //< 後方向
              if (canGo(v_fl, d_b)) {              //< 135度の壁
                const auto v_fll = v_fl.next(d_b); //< 前左左の区画
                if (canGo(v_fll, d_l))             //< 135度行先
                  pushAndContinue(Index(v_fll, d_f, nd_135),
                                  edge_cost.getEdgeCost(F135));
                if (canGo(v_fll, d_b)) //< 180度行先の壁
                  pushAndContinue(Index(v_fll, Dir::Max, nd_180),
                                  edge_cost.getEdgeCost(F180));
              }
            }
          }
        } else {
          /* 斜めなしのターン */
          const auto v_f = v.next(nd); //< i.e. vector front
          for (const auto d_turn : {Dir::Left, Dir::Right})
            if (canGo(v_f, nd + d_turn)) //< 90度方向の壁
              pushAndContinue(Index(v_f, Dir::Max, nd + d_turn),
                              edge_cost.getEdgeCost(FS90));
        }
      } else {
        /* 壁の中央（斜めありの場合しかありえない） */
        /* 直前の壁 */
        const auto i_f = focus.next(); //< i.e. index front
        if (!canGo(i_f.getVector(), i_f.getDir())) {
          loge << "FWE: " << focus << std::endl;
          continue;
        }
        /* 直進で行けるところまで行く */
        auto i_st = i_f; //< i.e. index straight
        for (int8_t n = 1;; ++n) {
          auto i_ff = i_st.next(); //< 行先の壁
          if (!canGo(i_ff.getVector(), i_ff.getDir()))
            break;
          if (!pushAndContinue(i_st, edge_cost.getEdgeCost(ST_DIAG, n)))
            break;
          i_st = i_ff;
        }
        /* ターン */
        auto nd_r45 = focus.arrow_diag_to_along_45();
        auto d_45 = nd + nd_r45;
        auto nd_90 = nd + nd_r45 * 2;
        auto d_135 = nd + nd_r45 * 3;
        auto v_45 = i_f.arrow_to();
        /* 45度方向 */
        if (canGo(v_45, d_45))
          pushAndContinue(Index(v_45, Dir::Max, d_45),
                          edge_cost.getEdgeCost(F45));
        /* V90方向, 135度方向*/
        if (canGo(v_45, d_135)) {
          /* V90方向, 135度方向*/
          auto v_135 = v_45.next(d_135);
          if (canGo(v_135, d_45))
            pushAndContinue(Index(v_45, d_135, nd_90),
                            edge_cost.getEdgeCost(FV90));
          if (canGo(v_135, d_135))
            pushAndContinue(Index(v_135, Dir::Max, d_135),
                            edge_cost.getEdgeCost(F135));
        }
      }
    }
  }
  static const Indexes convertDestinations(const Vectors src) {
    Indexes dest;
    for (const auto v : src)
      for (const auto nd : Dir::ENWS())
        dest.push_back(Index(v, Dir::Max, nd));
    return dest;
  }
  bool genPathFromMap(Indexes &path) {
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
  /**
   * @brief Get the Shortest Path Cost
   *        call calcShortestPath first.
   *
   * @return cost_t [ms]
   */
  cost_t getShortestPathCost() const { return f_map[index_start.opposite()]; }
  /**
   * @brief print the path on the maze
   *
   * @param indexes path
   * @param os out stream
   */
  void print(const Indexes indexes, std::ostream &os = std::cout) const;
  /**
   * @brief convert from indexes to dirs
   *
   * @param path
   * @param diag_enabled
   * @return const Dirs
   */
  static const Dirs indexes2dirs(const Indexes &path, const bool diag_enabled);

private:
  const Maze &maze; /**< @brief 使用する迷路の参照 */
  const EdgeCost edge_cost;
  const Index index_start =
      Index(0, 0, Dir::Max, Dir::North); /**< @brief start */

  std::function<bool(const Index i1, const Index i2)> greater;
  std::array<Index, Index::Max> from_map;
  std::vector<Index> open_list;
  std::array<cost_t, Index::Max> f_map;
  std::bitset<Index::Max> in_map;

public:
  int max_open_list_size = 0;
  int max_iteration_size = 0;
};

} // namespace MazeLib
