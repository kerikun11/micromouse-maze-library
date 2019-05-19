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

#include <algorithm> /*< for find_if, etc. */
#include <functional>
#include <iomanip> /*< for std::setw() */
#include <limits>  /*< for std::numeric_limits */
#include <unordered_map>

namespace MazeLib {

class ShortestAlgorithm {
public:
  ShortestAlgorithm(const Maze &maze) : maze(maze) {
    /* テーブルの事前計算 */
    getEdgeCost(ST_ALONG);
  }

public:
  using cost_t = uint16_t; /**< @brief 時間コストの型 [ms] */
  static constexpr cost_t CostMax = std::numeric_limits<cost_t>::max();
  enum Pattern : int8_t {
    ST_ALONG,
    ST_DIAG,
    F45,
    F90,
    F135,
    F180,
    FV90,
    FS90,
  };
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
  static cost_t getEdgeCost(const enum Pattern p, const int n = 1) {
    static std::array<cost_t, MAZE_SIZE * 2> cost_table_along;
    static std::array<cost_t, MAZE_SIZE * 2> cost_table_diag;
    static bool initialized = false; /*< 初回のみ実行するように設定 */
    if (!initialized) {
      initialized = true;
      /* 台形加速のコストテーブルを事前に用意 */
      static const float am = 3000.0f; /*< 最大加速度 [mm/s/s] */
      static const float vs = 450.0f;  /*< 終始速度 [mm/s] */
      static const float vm = 1800.0f; /*< 飽和速度 [mm/s] */
      for (int i = 0; i < MAZE_SIZE * 2; ++i) {
        const float d_along = 90.0f * (i + 1); /*< 走行距離 [mm] */
        const float d_diag = 1.41421356f * 45.0f * (i + 1); /*< 走行距離 [mm] */
        cost_table_along[i] =
            calcStraightTime(am, vs, vm, d_along) * 1000; /*< [ms] */
        cost_table_diag[i] =
            calcStraightTime(am, vs, vm, d_diag) * 1000; /*< [ms] */
      }
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
  /**
   * @brief Graph の Node の Index．
   * 「各区画中央の4方位」または「 各壁上の4方位」，の位置姿勢を一意に識別する
   */
  union __attribute__((__packed__)) Index {
  private:
    struct __attribute__((__packed__)) {
      int x : 6;           /**< @brief x coordinate of the cell */
      int y : 6;           /**< @brief y coordinate of the cell */
      unsigned int nd : 3; /**< @brief direction of the node */
      unsigned int
          z : 1; /**< @brief position assignment in the cell, 0:East; 1:North */
    };
    unsigned int all : 16; /**< @brief union element for all access */
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
     * @brief needed for unordered_map
     * uniqueなIDを返す
     */
    struct hash {
      size_t operator()(const Index &obj) const { return obj.all; }
    };
    /**
     * @brief 座標の冗長を一意にする．
     * d を East or North のどちらかにそろえる
     */
    void uniquify(const Dir d);
    const Dir getDir() const { return z == 0 ? Dir::East : Dir::North; }
    const Dir getNodeDir() const { return nd; }
    const Index &operator=(const Index &obj) { return all = obj.all, *this; }
    bool operator==(const Index &obj) const { return all == obj.all; }
    bool operator!=(const Index &obj) const { return all != obj.all; }
    operator Vector() const { return Vector(x, y); }
    friend std::ostream &operator<<(std::ostream &os, const Index &i) {
      return os << "( " << std::setw(2) << (int)i.x << ", " << std::setw(2)
                << (int)i.y << ", " << i.getDir().toChar() << ", "
                << i.getNodeDir().toChar() << ")";
    }
    const Vector arrow_from() const {
      switch (nd) {
      case Dir::East:
      case Dir::North:
      case Dir::West:
      case Dir::South:
        return Vector(x, y);
      case Dir::NorthEast:
        return Vector(x, y);
      case Dir::NorthWest:
        return z == 0 ? Vector(x + 1, y) : Vector(x, y);
      case Dir::SouthWest:
        return z == 0 ? Vector(x + 1, y) : Vector(x, y + 1);
      case Dir::SouthEast:
        return z == 0 ? Vector(x, y) : Vector(x, y + 1);
      default:
        break;
      }
      std::cerr << __FILE__ << ":" << __LINE__ << " "
                << "invalid direction" << std::endl;
      return Vector(x, y);
    }
    const Vector arrow_to() const {
      switch (nd) {
      case Dir::East:
      case Dir::North:
      case Dir::West:
      case Dir::South:
        return Vector(x, y).next(nd);
      case Dir::NorthEast:
        return z == 0 ? Vector(x + 1, y) : Vector(x, y + 1);
      case Dir::NorthWest:
        return z == 0 ? Vector(x, y) : Vector(x, y + 1);
      case Dir::SouthWest:
        return Vector(x, y);
      case Dir::SouthEast:
        return z == 0 ? Vector(x + 1, y) : Vector(x, y);
      default:
        break;
      }
      std::cerr << __FILE__ << ":" << __LINE__ << " "
                << "invalid direction" << std::endl;
      return Vector(x, y);
    }
    /**
     * @brief 斜め方向に向いているときの区画への相対方向(±45度)を返す
     * @return const Dir Dir::Left45 or Dir::Right45
     */
    const Dir arrow_diag_to_along_45() const {
      switch (nd) {
      case Dir::NorthEast:
      case Dir::SouthWest:
        return z == 0 ? Dir::Left45 : Dir::Right45;
      case Dir::NorthWest:
      case Dir::SouthEast:
        return z == 1 ? Dir::Left45 : Dir::Right45;
      }
      std::cerr << __FILE__ << ":" << __LINE__ << " "
                << "invalid direction" << std::endl;
      return Dir::AbsMax;
    }
    /**
     * @brief NodeDir が向いている方向の隣の Index を返す
     * @return const Index
     */
    const Index next() const {
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
      std::cerr << __FILE__ << ":" << __LINE__ << " "
                << "invalid direction" << std::endl;
      return Index();
    }
    const Index opposite() const {
      return Index(x, y, getDir(), nd + Dir::Back);
    }
    void successors_for(
        const Maze &maze, const bool known_only, const bool diag_enabled,
        std::function<void(const Index, const cost_t)> callback) const;
    void predecessors_for(
        const Maze &maze, const bool known_only, const bool diag_enabled,
        std::function<void(const Index, const cost_t)> callback) const;
  };
  static_assert(sizeof(Index) == 2, "Index Size Error"); /**< Size Check */
  typedef std::vector<Index> Indexes;

  /**
   * @brief Graph の Node
   */
  struct __attribute__((__packed__)) Node {
    cost_t cost = CostMax;
    Node() {}
  };
  static_assert(sizeof(Node) == 2, "Node Size Error"); /**< Size Check */

  /**
   * @brief Get the Heuristic Value
   * @param i Index
   * @return cost_t heuristic value
   */
  cost_t getHeuristic(const Index i) const {
    return 0;
    const auto v = Vector(i) - Vector(index_start);
    // const float d = std::sqrt(v.x * v.x + v.y * v.y);
    const auto d = std::max(std::abs(v.x), std::abs(v.y));
    return getEdgeCost(ST_ALONG, d);
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
                        const bool diag_enabled) {
    std::unordered_map<Index, Node, Index::hash> node_map;
    std::function<bool(const Index &i1, const Index &i2)> greater =
        [&](const auto &i1, const auto &i2) {
          return node_map[i1].cost > node_map[i2].cost;
        };
    std::vector<Index> open_list;
    /* clear open_list */
    open_list.clear();
    std::make_heap(open_list.begin(), open_list.end(), greater);
    /* clear node map */
    node_map.clear();
    /* push the goal indexes */
    for (const auto v : maze.getGoals())
      for (const auto nd : Dir::ENWS()) {
        const auto i = Index(v, Dir::AbsMax, nd);
        node_map[i].cost = 0;
        open_list.push_back(i);
        std::push_heap(open_list.begin(), open_list.end(), greater);
      }
    /* ComputeShortestPath() */
    while (1) {
      // std::cout << "size():\t" << open_list.size() << std::endl;
      if (open_list.empty()) {
        std::cerr << "open_list.empty()" << std::endl;
        return false;
      }
      /* place the element with a min cost to back */
      std::pop_heap(open_list.begin(), open_list.end(), greater);
      const auto index = open_list.empty()
                             ? Index(-1, -1, Dir::AbsMax, Dir::AbsMax)
                             : open_list.back();
      open_list.pop_back();
      /* 終了条件 */
      if (index == index_start)
        break;
      index.successors_for(
          maze, known_only, diag_enabled,
          [&](const auto i_succ, const auto edge_cost __attribute__((unused))) {
            Node &succ = node_map[i_succ];
            cost_t h_n = getHeuristic(index);
            cost_t h_m = getHeuristic(i_succ);
            cost_t g_n = node_map[index].cost - h_n;
            cost_t f_m_prime = g_n + edge_cost + h_m;
            if (f_m_prime < succ.cost) {
              succ.cost = f_m_prime;
          /* remove i from open_list */
#if 0
              open_list.erase(
                  std::remove(open_list.begin(), open_list.end(), nibr_index),
                  open_list.end());
              std::make_heap(open_list.begin(), open_list.end(), greater);
#endif
              open_list.push_back(i_succ);
              std::push_heap(open_list.begin(), open_list.end(), greater);
            }
          });
    }
    /* post process */
    path.erase(path.begin(), path.end());
    auto i = index_start;
    while (1) {
      // std::cout << i << std::endl;
      path.push_back(i.opposite());
      if (node_map[i].cost == 0)
        break;
      /* 最小コストの方向を探す */
      auto min_cost = CostMax;
      auto next = i;
      i.predecessors_for(
          maze, known_only, diag_enabled,
          [&](const auto pre, const auto cost __attribute__((unused))) {
            const auto cost_p = node_map[pre].cost;
            if (cost_p < min_cost) {
              min_cost = cost_p;
              next = pre;
            }
          });
      if (next == i)
        return false;
      i = next;
    }
    return true;
  }
  void printPath(std::ostream &os, const Indexes indexes) const;
  static const Dirs indexes2dirs(const Indexes &path, const bool diag_enabled);

private:
  const Maze &maze; /**< @brief 使用する迷路の参照 */
  const Index index_start =
      Index(0, 0, Dir::AbsMax, Dir::South); /**< @brief スタート */
};

} // namespace MazeLib
