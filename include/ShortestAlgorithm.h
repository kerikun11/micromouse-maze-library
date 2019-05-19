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

#include "log.h"

#include <algorithm> /*< for find_if, etc. */
#include <functional>
#include <iomanip> /*< for std::setw() */
#include <limits>  /*< for std::numeric_limits */
#include <unordered_map>

namespace MazeLib {

class ShortestAlgorithm {
public:
  ShortestAlgorithm(const Maze &maze, const bool diag_enabled)
      : maze(maze), diag_enabled(diag_enabled),
        greater([&](const auto &i1, const auto &i2) {
          auto m1 = std::min(node_map[i1].cost, node_map[i1].rhs);
          auto m2 = std::min(node_map[i2].cost, node_map[i2].rhs);
          return m1 > m2;
        }) {
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
      logw << "Invalid Index" << std::endl;
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
      logw << "Invalid Index" << std::endl;
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
      logw << "Invalid Node Dir" << std::endl;
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
      logw << "Invalid Index" << std::endl;
      return Index();
    }
    const Index opposite() const {
      return Index(x, y, getDir(), nd + Dir::Back);
    }
    void successors_for(const Maze &maze, const bool known_only,
                        const bool diag_enabled,
                        std::function<void(const Index, const cost_t)> callback,
                        const bool ignore_front_wall = false) const;
    void predecessors_for(
        const Maze &maze, const bool known_only, const bool diag_enabled,
        std::function<void(const Index, const cost_t)> callback) const;
  };
  static_assert(sizeof(Index) == 2, "Index Size Error"); /**< Size Check */
  typedef std::vector<Index> Indexes;

  /**
   * @brief Key
   */
  using HeapKey = std::pair<cost_t, cost_t>;
  struct KeyLess {
    bool operator()(const std::pair<HeapKey, unsigned int> &k1,
                    const std::pair<HeapKey, unsigned int> &k2) const {
      return k1.first < k2.first;
    }
  };
  /**
   * @brief Graph の Node
   */
  struct __attribute__((__packed__)) Node {
    cost_t cost = CostMax;
    cost_t rhs = CostMax;
    Node() {}
  };
  static_assert(sizeof(Node) == 4, "Node Size Error"); /**< Size Check */

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
  void Initialize() {
    /* init */
    wall_log_count = 0;
    /* clear open_list */
    open_list.clear();
    std::make_heap(open_list.begin(), open_list.end(), greater);
    /* clear node map */
    node_map.clear();
    /* push the goal indexes */
    for (const auto v : maze.getGoals())
      for (const auto nd : Dir::ENWS()) {
        const auto i = Index(v, Dir::AbsMax, nd);
        node_map[i].rhs = 0;
        open_list.push_back(i);
        std::push_heap(open_list.begin(), open_list.end(), greater);
      }
    // Indexes path;
    // calcShortestPath(path, diag_enabled);
  }
  void UpdateNode(const Index i, const bool known_only) {
    auto &node = node_map[i];
    // std::cout << "update\t" << i << "\t" << node.rhs << "\t" << node.cost
    //           << std::endl;
    if (node.rhs == 0)
      return;
    node.rhs = CostMax; /* update */
    const auto h_n = getHeuristic(i);
    i.predecessors_for(maze, known_only, diag_enabled,
                       [&](const auto i_pre, const auto edge_cost) {
                         const auto &pre = node_map[i_pre];
                         const auto h_p = getHeuristic(i_pre);
                         const auto new_cost = pre.cost - h_p + edge_cost + h_n;
                         if (new_cost < node.rhs)
                           node.rhs = new_cost;
                       });
    /* remove i from open_list */
    // open_list.erase(std::remove(open_list.begin(), open_list.end(), i),
    //                 open_list.end());
    // std::make_heap(open_list.begin(), open_list.end(), greater);
    if (node.cost != node.rhs) {
      open_list.push_back(i);
      std::push_heap(open_list.begin(), open_list.end(), greater);
      // std::cout << "push\t" << i << "\t" << node.rhs << "\t" << node.cost
      //           << std::endl;
    } else {
      // std::cout << "no push\t" << i << "\t" << node.rhs << "\t" <<
      // node.cost
      //           << std::endl;
    }
  }
  void UpdateChangedEdge(const bool known_only) {
    /* wall log */
    const int maze_wall_log_size = maze.getWallLogs().size();
    if (wall_log_count >= maze_wall_log_size)
      return;
    /* 各WallLogに対して */
    while (wall_log_count < maze_wall_log_size) {
      const auto wl = maze.getWallLogs()[wall_log_count++];
      const auto w_v = Vector(wl);
      const auto w_d = Dir(wl);
      /* 壁があった場合のみ処理 */
      if (wl.b) {
        // std::cout << "find:\t" << wl << std::endl;
        if (diag_enabled) {
          for (const auto nd : Dir::Diag4()) {
            auto i = Index(wl.x, wl.y, wl.d, nd);
            UpdateNode(i, known_only); /* update */
            i.successors_for(maze, known_only, diag_enabled,
                             [&](const auto i_succ,
                                 const auto edge_cost __attribute__((unused))) {
                               //  const auto &node = node_map[i_succ];
                               //  std::cout << "succ w\t" << i_succ << "\t"
                               //            << node.rhs << "\t" << node.cost
                               //            << std::endl;
                               UpdateNode(i_succ, known_only); /* update */
                             },
                             true);
          }
        }
        /* 壁ができてしまったので更新されないバグがある */
        for (const auto i :
             {Index(w_v, Dir::AbsMax, w_d),
              Index(w_v.next(w_d), Dir::AbsMax, w_d + Dir::Back)}) {
          UpdateNode(i, known_only); /* update */
          i.successors_for(maze, known_only, diag_enabled,
                           [&](const auto i_succ,
                               const auto edge_cost __attribute__((unused))) {
                             //  const auto &node = node_map[i_succ];
                             //  std::cout << "succ c\t" << i_succ << "\t"
                             //            << node.rhs << "\t" << node.cost
                             //            << std::endl;
                             UpdateNode(i_succ, known_only); /* update */
                           },
                           true);
        }
      }
    }
  }
  /**
   * @brief 最短経路を求める
   *
   * @param path 結果を入れる箱
   * @param known_only
   * @return true 成功
   * @return false 失敗
   */
  bool calcShortestPath(Indexes &path, const bool known_only) {
    // Initialize();
    /* changed */
    UpdateChangedEdge(known_only);
    /* util */
    const auto &start = node_map[index_start];
    /* ComputeShortestPath() */
    while (1) {
      // std::cout << "size():\t" << open_list.size() << std::endl;
      if (open_list.empty()) {
        std::cerr << "open_list.empty()" << std::endl;
        // return false;
        break;
      }
      /* place the element with a min cost to back */
      std::pop_heap(open_list.begin(), open_list.end(), greater);
      const auto index = open_list.empty()
                             ? Index(-1, -1, Dir::AbsMax, Dir::AbsMax)
                             : open_list.back();
      open_list.pop_back();
      auto &node = node_map[index];
      /* 終了条件 */
      if (!(greater(index_start, index) || start.cost != start.rhs))
        break;
      if (node.cost > node.rhs) {
        // std::cout << "g < r\t" << index << "\t" << node.rhs << "\t" <<
        // node.cost
        //           << std::endl;
        node.cost = node.rhs; /* update */
        index.successors_for(maze, known_only, diag_enabled,
                             [&](const auto i_succ,
                                 const auto edge_cost __attribute__((unused))) {
                               UpdateNode(i_succ, known_only); /* update */
                             });
      } else if (node.cost < node.rhs) {
        // std::cout << "g > r\t" << index << "\t" << node.rhs << "\t" <<
        // node.cost
        //           << std::endl;
        node.cost = CostMax;           /* update */
        UpdateNode(index, known_only); /* update */
        index.successors_for(maze, known_only, diag_enabled,
                             [&](const auto i_succ,
                                 const auto edge_cost __attribute__((unused))) {
                               UpdateNode(i_succ, known_only); /* update */
                             });
      } else {
        // std::cout << "g == r\t" << index << "\t" << node.rhs << "\t"
        //           << node.cost << std::endl;
      }
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
  const bool diag_enabled;
  const Index index_start =
      Index(0, 0, Dir::AbsMax, Dir::South); /**< @brief スタート */
  std::unordered_map<Index, Node, Index::hash> node_map;
  std::function<bool(const Index &i1, const Index &i2)> greater;
  std::vector<Index> open_list;
  int wall_log_count = 0;
};

} // namespace MazeLib
