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

namespace MazeLib {

using cost_t = uint16_t; /**< @brief 時間コストの型 [ms] */
static constexpr cost_t CostMax = std::numeric_limits<cost_t>::max();
/**
 * @brief 最短走行パターン
 */
enum Pattern : int8_t { ST_ALONG, ST_DIAG, F45, F90, F135, F180, FV90, FS90 };

/**
 * @brief 最短走行アクション
 */
struct Action {
  enum Type : int8_t {
    Straight,
    Slalom,
    TypeMax,
  };
  enum StraightDirection : int8_t {
    Along,
    Diag,
    StraightDirectionMax,
  };
  enum SlalomDirection : int8_t {
    Left,
    Right,
    SlalomDirectionMax,
  };
  enum SlalomIndex : int8_t {
    F45,
    F90,
    F135,
    F180,
    FV90,
    FS90,
    SlalomMax,
  };
  Type type;
  int8_t direction;
  int8_t index;
  friend std::ostream &operator<<(std::ostream &os, const Type type) {
    switch (type) {
    case Straight:
      return os << "S";
    case Slalom:
      return os << "T";
    default:
      return os;
    }
  }
};

struct RunParameter {
  std::array<float, Action::SlalomMax> v_slalom;
  std::array<float, Action::StraightDirectionMax> v_max;
  std::array<float, Action::StraightDirectionMax> a_max;
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
    default:
      logw << "Unknown Pattern" << std::endl;
      return 0;
    }
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
  /** @brief デフォルトコンストラクタ */
  Index() : all(0) {}
  /** @brief 成分を受け取ってそのまま代入するコンストラクタ */
  Index(const int8_t x, const int8_t y, const uint8_t z, const Direction nd)
      : x(x), y(y), z(z), nd(nd) {}
  /** @brief 冗長性を除去するコンストラクタ */
  Index(const Position &p, const Direction d, const Direction nd)
      : x(p.x), y(p.y), nd(nd) {
    uniquify(d);
  }
  // Index(const int8_t x, const int8_t y, const Direction d, const Direction
  // nd)
  //     : x(x), y(y), nd(nd) {
  //   uniquify(d);
  // }
  /** @brief 区画中央のコンストラクタ */
  Index(const int8_t x, const int8_t y, const Direction nd)
      : x(x), y(y), z(0), nd(nd) {}
  Index(const Position &p, const Direction nd) : x(p.x), y(p.y), z(0), nd(nd) {}
  /** @brief 壁上のコンストラクタ */
  Index(const WallIndex &i, const Direction nd)
      : x(i.x), y(i.y), z(i.z), nd(nd) {}
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
  void uniquify(const Direction d);
  /**
   * @brief Getters
   */
  const Direction getDirection() const {
    return z == 0 ? Direction::East : Direction::North;
  }
  const Direction getNodeDirection() const { return nd; }
  const Position getPosition() const { return Position(x, y); }
  friend std::ostream &operator<<(std::ostream &os, const Index &i) {
    return os << "( " << std::setw(2) << (int)i.x << ", " << std::setw(2)
              << (int)i.y << ", " << i.getDirection().toChar() << ", "
              << i.getNodeDirection().toChar() << ")";
  }
  const Position arrow_from() const;
  const Position arrow_to() const;
  /**
   * @brief 斜め方向に向いているときの区画への相対方向(±45度)を返す
   * @return const Direction Direction::Left45 or Direction::Right45
   */
  const Direction arrow_diag_to_along_45() const;
  /**
   * @brief NodeDirection が向いている方向の隣の Index を返す
   * @return const Index
   */
  const Index next() const;
  const Index opposite() const { return Index(x, y, z, nd + Direction::Back); }
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
  cost_t getHeuristic(const Index &i) const {
    return getHeuristic(i, index_start);
  }
  cost_t getHeuristic(const Index &i, const Index &s) const {
    // return 0;
    const auto p = i.getPosition() - s.getPosition();
    // const auto d = std::sqrt(p.x * p.x + p.y * p.y);
    const auto d = std::max(std::abs(p.x), std::abs(p.y));
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
  static const Indexes convertDestinations(const Positions src) {
    Indexes dest;
    for (const auto p : src)
      for (const auto nd : Direction::getAlong4())
        dest.push_back(Index(p, nd));
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
   * @return const Directions
   */
  static const Directions indexes2directions(const Indexes &path,
                                             const bool diag_enabled);

private:
  const Maze &maze;
  const EdgeCost edge_cost;
  const Index index_start = Index(0, 0, Direction::North);

  std::function<bool(const Index &i1, const Index &i2)> greater;
  std::array<Index, Index::Max> from_map;
  std::vector<Index> open_list;
  std::array<cost_t, Index::Max> f_map;
  std::bitset<Index::Max> in_map;

public:
  int max_open_list_size = 0;
  int max_iteration_size = 0;
};

} // namespace MazeLib
