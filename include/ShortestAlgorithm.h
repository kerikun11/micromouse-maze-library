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
  ShortestAlgorithm(const Maze &maze)
      : maze(maze), greater([&](const auto &i1, const auto &i2) {
          const auto h1 = std::min(g_map[i1], rhs_map[i1]) + getHeuristic(i1);
          const auto h2 = std::min(g_map[i2], rhs_map[i2]) + getHeuristic(i2);
          if (h1 != h2)
            return h1 > h2;
          const auto m1 = std::min(g_map[i1], rhs_map[i1]);
          const auto m2 = std::min(g_map[i2], rhs_map[i2]);
          return m1 > m2;
        }) {
    /* テーブルの事前計算 */
    getEdgeCost(ST_ALONG);
  }

public:
  using cost_t = uint16_t; /**< @brief 時間コストの型 [ms] */
  static constexpr cost_t CostMax = std::numeric_limits<cost_t>::max();
  /**
   * @brief 最短走行パターン
   */
  enum Pattern : int8_t { ST_ALONG, ST_DIAG, F45, F90, F135, F180, FV90, FS90 };
  /**
   * @brief Get the Edge Cost
   * @param p パターン
   * @param n 直線の場合，区画数
   * @return cost_t コスト
   */
  static cost_t getEdgeCost(const enum Pattern p, const int n = 1);
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
    static constexpr int Max = MAZE_SIZE * MAZE_SIZE * 16;

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
     * unique な ID を返す
     */
    struct hash {
      size_t operator()(const Index &obj) const { return obj.all; }
    };
    operator uint16_t() const {
      return (nd << 11) | (z << 10) | (y << 5) | x; /*< M * M * 16 */
      // return (((~nd) & 1) << 13) | (z << 12) | ((6 & nd) << 9) | (x << 5) |
      //        y; /*< M * M * 12 */
    }
    /**
     * @brief 座標の冗長を一意にする．
     * d を East or North のどちらかにそろえる
     */
    void uniquify(const Dir d);
    const Dir getDir() const { return z == 0 ? Dir::East : Dir::North; }
    const Dir getNodeDir() const { return nd; }
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
    const std::vector<std::pair<Index, cost_t>>
    getSuccessors(const Maze &maze, const bool known_only,
                  const bool diag_enabled) const;
    const std::vector<std::pair<Index, cost_t>>
    getPredecessors(const Maze &maze, const bool known_only,
                    const bool diag_enabled) const;
  };
  static_assert(sizeof(Index) == 2, "Index Size Error"); /**< Size Check */
  typedef std::vector<Index> Indexes;

  /**
   * @brief Get the Heuristic Value
   * @param i Index
   * @return cost_t heuristic value
   */
  cost_t getHeuristic(const Index i) const {
    // return 0;
    const auto v = Vector(i) - Vector(index_start);
    // const auto d = std::sqrt(v.x * v.x + v.y * v.y);
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

  void UpdateNode(const Index i, const bool known_only,
                  const bool diag_enabled) {
    if (rhs_map[i] == 0)
      return;
    rhs_map[i] = CostMax;
    const auto preds = i.getPredecessors(maze, known_only, diag_enabled);
    for (const auto &p : preds) {
      if (!Vector(p.first).isInsideOfField())
        std::cerr << __FILE__ << ":" << __LINE__ << " "
                  << "Warning! " << p.first << std::endl;
      if (g_map[p.first] != CostMax)
        rhs_map[i] = std::min(rhs_map[i], (cost_t)(g_map[p.first] + p.second));
    }
    if (g_map[i] != rhs_map[i]) {
      open_list.push_back(i);
      std::push_heap(open_list.begin(), open_list.end(), greater);
    }
  };
  void UpdateChangedEdge(const bool known_only, const bool diag_enabled);
  void Initialize() {
    /* clear open_list */
    open_list.clear();
    std::make_heap(open_list.begin(), open_list.end(), greater);
    /* clear node map */
    for (auto &node : g_map)
      node = CostMax;
    for (auto &node : rhs_map)
      node = CostMax;
    /* push the goal indexes */
    for (const auto v : maze.getGoals())
      for (const auto nd : Dir::ENWS()) {
        const auto i = Index(v, Dir::AbsMax, nd);
        rhs_map[i] = 0;
        open_list.push_back(i);
        std::push_heap(open_list.begin(), open_list.end(), greater);
      }
  }
  bool ComputeShortestPath(const bool known_only, const bool diag_enabled) {
    while (1) {
      // std::cout << "size():\t" << open_list.size() << std::endl;
      if (open_list.empty()) {
        std::cerr << "open_list.empty()" << std::endl;
        return false;
      }
      /* place the element with the min cost to back */
      std::pop_heap(open_list.begin(), open_list.end(), greater);
      const auto index = open_list.back();
      open_list.pop_back();
      /* breaking condition */
      if (!(greater(index_start, index) ||
            rhs_map[index_start] != g_map[index_start]))
        break;
      if (g_map[index] > rhs_map[index]) {
        g_map[index] = rhs_map[index];
        const auto succs = index.getSuccessors(maze, known_only, diag_enabled);
        for (const auto &s : succs) {
          if (!Vector(s.first).isInsideOfField())
            std::cerr << __FILE__ << ":" << __LINE__ << " "
                      << "Warning! " << s.first << std::endl;
          UpdateNode(s.first, known_only, diag_enabled);
        }
      } else if (g_map[index] < rhs_map[index]) {
        g_map[index] = CostMax;
        UpdateNode(index, known_only, diag_enabled);
        const auto succs = index.getSuccessors(maze, known_only, diag_enabled);
        for (const auto &s : succs) {
          if (!Vector(s.first).isInsideOfField())
            std::cerr << __FILE__ << ":" << __LINE__ << " "
                      << "Warning! " << s.first << std::endl;
          UpdateNode(s.first, known_only, diag_enabled);
        }
      }
    }
    return true;
  }
  bool calcShortestPath(Indexes &path, const bool known_only,
                        const bool diag_enabled);
  void printPath(std::ostream &os, const Indexes indexes) const;
  static const Dirs indexes2dirs(const Indexes &path, const bool diag_enabled);

private:
  const Maze &maze; /**< @brief 使用する迷路の参照 */
  const Index index_start =
      Index(0, 0, Dir::AbsMax, Dir::South); /**< @brief スタート */
  std::array<cost_t, Index::Max> g_map;
  std::array<cost_t, Index::Max> rhs_map;
  std::vector<Index> open_list;
  int wall_log_count = 0;
  std::function<bool(const Index &i1, const Index &i2)> greater;
};

} // namespace MazeLib
