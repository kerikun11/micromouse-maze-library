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
  using cost_t = uint16_t; /**< @brief 時間コストの型 [ms] */
  static constexpr cost_t CostMax = std::numeric_limits<cost_t>::max() - 4096;
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
#define INDEX_ARRANGEMENT 2
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
      return (((~nd) & 1) << 13) | (z << 12) | ((6 & nd) << 9) | (x << 5) |
             y; /*< M * M * 12 */
#endif
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
      loge << "invalid direction" << std::endl;
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
      loge << "invalid direction" << std::endl;
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
      loge << "invalid direction" << std::endl;
      return Dir::AbsMax;
    }
    /**
     * @brief NodeDir が向いている方向の隣の Index を返す
     * @return const Index
     */
    const Index next() const;
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
    return getHeuristic(i, index_start);
  }
  cost_t getHeuristic(const Index i, const Index s) const {
    // return 0;
    const auto v = Vector(i) - Vector(s);
    // const auto d = std::sqrt(v.x * v.x + v.y * v.y);
    const auto d = std::max(std::abs(v.x), std::abs(v.y));
    return getEdgeCost(ST_ALONG, d);
  }
  using Key = std::pair<cost_t, cost_t>;
  struct KeyCompare {
    bool operator()(const std::pair<Index, Key> &k1,
                    const std::pair<Index, Key> &k2) const {
      return k1.second > k2.second;
    }
  };
  const Key CalculateKey(const Index s) const {
    const auto m = std::min(g_map[s], r_map[s]);
    return {m + getHeuristic(s) + k_m, m};
  }
  void Initialize() {
    U.clear();
    std::make_heap(U.begin(), U.end(), KeyCompare());
    k_m = 0;
    for (int i = 0; i < Index::Max; ++i)
      r_map[i] = g_map[i] = CostMax;
    for (const auto v : maze.getGoals())
      for (const auto nd : Dir::ENWS()) {
        const auto i = Index(v, Dir::AbsMax, nd);
        r_map[i] = 0;
        U.push_back({i, CalculateKey(i)});
        std::push_heap(U.begin(), U.end(), KeyCompare());
        in_map[i] = true;
      }
    wall_log_count = 0;
  }
  void UpdateVertex(const Index u, const bool known_only,
                    const bool diag_enabled) {
    if (r_map[u] != 0) {
      const auto successors = u.getSuccessors(maze, known_only, diag_enabled);
      /* min_element */
      auto min_g = CostMax;
      for (const auto &s_prime : successors) {
        if (!Vector(s_prime.first).isInsideOfField())
          std::cerr << __FILE__ << ":" << __LINE__ << " "
                    << "Warning! " << s_prime.first << std::endl;
        const auto new_g = s_prime.second + g_map[s_prime.first];
        if (min_g > new_g) {
          min_g = new_g;
          from_map[u] = s_prime.first;
        }
      }
      r_map[u] = min_g;
    }
    if (in_map[u]) {
      U.erase(std::find_if(U.cbegin(), U.cend(),
                           [u](const auto &e) { return e.first == u; }));
      in_map[u] = false;
    }
    if (g_map[u] != r_map[u]) {
      U.push_back({u, CalculateKey(u)});
      std::push_heap(U.begin(), U.end(), KeyCompare());
      in_map[u] = true;
    }
  }
  void UpdateChangedEdge(const bool known_only, const bool diag_enabled) {
    /* wall log */
    const int maze_wall_log_size = maze.getWallLogs().size();
    /* 各WallLogに対して */
    while (wall_log_count < maze_wall_log_size) {
      const auto wl = maze.getWallLogs()[wall_log_count++];
      const auto w_v = Vector(wl);
      const auto w_d = Dir(wl);
      /* 壁があった場合のみ処理 */
      if (wl.b) {
        // std::cout << "find: " << wl << std::endl;
        if (diag_enabled) {
          for (const auto nd : Dir::Diag4()) {
            const auto i = Index(wl.x, wl.y, wl.d, nd);
            UpdateVertex(i, known_only, diag_enabled);
            for (const auto s :
                 i.getSuccessors(maze, known_only, diag_enabled)) {
              if (!Vector(s.first).isInsideOfField())
                std::cerr << __FILE__ << ":" << __LINE__ << " "
                          << "Warning! " << s.first << std::endl;
              UpdateVertex(s.first, known_only, diag_enabled);
              UpdateVertex(s.first.opposite(), known_only, diag_enabled);
            }
            for (const auto s :
                 i.next().getSuccessors(maze, known_only, diag_enabled)) {
              if (!Vector(s.first).isInsideOfField())
                std::cerr << __FILE__ << ":" << __LINE__ << " "
                          << "Warning! " << s.first << std::endl;
              UpdateVertex(s.first, known_only, diag_enabled);
              UpdateVertex(s.first.opposite(), known_only, diag_enabled);
            }
          }
        }
        for (const auto i : {
                 Index(w_v, Dir::AbsMax, w_d + Dir::Back),
                 Index(w_v.next(w_d), Dir::AbsMax, w_d),
             }) {
          UpdateVertex(i, known_only, diag_enabled);
          UpdateVertex(i.opposite(), known_only, diag_enabled);
          for (const auto s : i.getSuccessors(maze, known_only, diag_enabled)) {
            if (!Vector(s.first).isInsideOfField())
              std::cerr << __FILE__ << ":" << __LINE__ << " "
                        << "Warning! " << s.first << std::endl;
            UpdateVertex(s.first, known_only, diag_enabled);
            UpdateVertex(s.first.opposite(), known_only, diag_enabled);
          }
        }
      }
    }
  }
  bool ComputeShortestPath(const bool known_only, const bool diag_enabled) {
    for (int j = 0;; ++j) {
      std::make_heap(U.begin(), U.end(), KeyCompare());
      std::pop_heap(U.begin(), U.end(), KeyCompare());
      auto top = U.back();
      if (U.empty())
        top.second = std::pair<cost_t, cost_t>{CostMax, CostMax};
      if (!(top.second < CalculateKey(index_start) ||
            r_map[index_start] != g_map[index_start]))
        break;
      if (U.empty()) {
        logw << "U.empty()" << std::endl;
        break;
      }
      const auto k_old = top.second;
      const auto u = top.first;
      U.pop_back();
      in_map[u] = false;
      if (k_old < CalculateKey(u)) {
        U.push_back({u, CalculateKey(u)});
        std::push_heap(U.begin(), U.end(), KeyCompare());
        in_map[u] = true;
      } else if (g_map[u] > r_map[u]) {
        g_map[u] = r_map[u];
        const auto predecessors =
            u.getPredecessors(maze, known_only, diag_enabled);
        for (const auto &s : predecessors) {
          if (!Vector(s.first).isInsideOfField())
            std::cerr << __FILE__ << ":" << __LINE__ << " "
                      << "Warning! " << s.first << std::endl;
          UpdateVertex(s.first, known_only, diag_enabled);
        }
      } else {
        g_map[u] = CostMax;
        UpdateVertex(u, known_only, diag_enabled);
        const auto predecessors =
            u.getPredecessors(maze, known_only, diag_enabled);
        for (const auto &s : predecessors) {
          if (!Vector(s.first).isInsideOfField())
            std::cerr << __FILE__ << ":" << __LINE__ << " "
                      << "Warning! " << s.first << std::endl;
          UpdateVertex(s.first, known_only, diag_enabled);
        }
      }
    }
    return true;
  }
  bool FollowShortestPath(Indexes &path, const bool known_only,
                          const bool diag_enabled) const {
    /* post process */
    path.erase(path.begin(), path.end());
    auto i = index_start;
    while (1) {
      // std::cout << i << "\t" << g_map[i] << std::endl;
      path.push_back(i);
      if (g_map[i] == 0)
        break;
#if 0
      i = from_map[i];
#else
      /* find the index with the min cost */
      auto g_min = CostMax;
      auto next = i;
      const auto successors = i.getSuccessors(maze, known_only, diag_enabled);
      for (const auto &p : successors) {
        if (!Vector(p.first).isInsideOfField())
          loge << "Out of Range! " << p.first << std::endl;
        const auto g_p = g_map[p.first] + p.second;
        if (g_min > g_p) {
          g_min = g_p;
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
  // bool main(const bool known_only, const bool diag_enabled) {
  //   auto s_start = index_start;
  //   auto s_last = s_start;
  //   Initialize();
  //   ComputeShortestPath(known_only, diag_enabled);
  //   while (r_map[s_start] != 0) {
  //     if (g_map[index_start] == CostMax)
  //       return false;
  //     std::cout << "\e[0;0H"; //< カーソルを左上に移動
  //     printPath(std::cout, {s_start});
  //     /* 行ける方向を探す */
  //     const auto successors =
  //         s_start.getSuccessors(maze, known_only, diag_enabled);
  //     auto min_g = CostMax;
  //     for (const auto &s_prime : successors) {
  //       if (!Vector(s_prime.first).isInsideOfField())
  //         loge << "Out of Range! " << s_prime.first << std::endl;
  //       const auto new_g = s_prime.second + g_map[s_prime.first];
  //       if (min_g > new_g) {
  //         min_g = new_g;
  //         s_start = s_prime.first;
  //       }
  //     }
  //     /* Move to s_start */
  //     if (1) {
  //       k_m = k_m + getHeuristic(s_last, s_start);
  //       s_last = s_start;
  //       UpdateChangedEdge(known_only, diag_enabled);
  //       ComputeShortestPath(known_only, diag_enabled);
  //     }
  //   }
  //   return true;
  // }
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
   * @brief print the path on the maze
   *
   * @param os out stream
   * @param indexes path
   */
  void printPath(std::ostream &os, const Indexes indexes) const;
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
  const Index index_start =
      Index(0, 0, Dir::AbsMax, Dir::North); /**< @brief スタート */

  std::vector<std::pair<Index, Key>> U;
  std::array<cost_t, Index::Max> g_map;
  std::array<cost_t, Index::Max> r_map;
  std::bitset<Index::Max> in_map;
  cost_t k_m;
  int wall_log_count;

  std::function<bool(const Index i1, const Index i2)> greater;
  std::array<Index, Index::Max> from_map;
  std::vector<Index> open_list;
  std::array<cost_t, Index::Max> f_map;

public:
  int max_open_list_size = 0;
  int max_iteration_size = 0;
};

} // namespace MazeLib
