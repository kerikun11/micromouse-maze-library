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
#include <queue>
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
    Index(const Index &obj) : all(obj.all) {}
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
    void uniquify(const Dir d) {
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
        std::cerr << "Invalid Direction" << std::endl;
        break;
      }
    }
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
      std::cerr << "Invalid Index" << std::endl;
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
      std::cerr << "Invalid Index" << std::endl;
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
      std::cerr << "Invalid Node Dir" << std::endl;
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
      std::cerr << "Invalid Index" << std::endl;
      return Index();
    }
    void neighbors_for(
        const Maze &maze, const bool known_only, const bool diag_enabled,
        std::function<void(const Index, const cost_t)> callback) const {
      /* known_only を考慮した壁の判定式を用意 */
      auto canGo = [&](const Vector vec, const Dir dir) {
        if (maze.isWall(vec, dir))
          return false;
        if (known_only && !maze.isKnown(vec, dir))
          return false;
        return true;
      };
      if (!diag_enabled) {
        /* 直進で行けるところまで行く */
        auto v_st = arrow_to(); //< 前方のマス
        for (int8_t n = 1;; n++) {
          if (!canGo(v_st, nd))
            break;
          callback(Index(v_st, Dir::AbsMax, nd), getEdgeCost(ST_ALONG, n));
          v_st = v_st.next(nd);
        }
        /* ここからはターン */
        const auto v_f = arrow_to(); //< i.e. vector front
        /* 左右を一般化 */
        for (const auto d_turn : {Dir::Left, Dir::Right})
          if (canGo(v_f, nd + d_turn)) //< 90度方向の壁
            callback(Index(v_f, Dir::AbsMax, nd + d_turn), getEdgeCost(FS90));
        return;
      }
      /* 区画の中央 */
      if (Dir(nd).isAlong()) {
        Vector v = Vector(x, y); //< 仮想位置
        /* 直前の壁 */
        if (!canGo(v, nd)) {
          std::cerr << "Something Wrong" << std::endl;
          return;
        }
        /* 直進で行けるところまで行く */
        auto v_st = v.next(nd);
        for (int8_t n = 1;; n++) {
          if (!canGo(v_st, nd))
            break;
          callback(Index(v_st, Dir::AbsMax, nd), getEdgeCost(ST_ALONG, n));
          v_st = v_st.next(nd);
        }
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
          std::cerr << "Something Wrong" << std::endl;
          return;
        }
        /* 直進で行けるところまで行く */
        auto i_st = i_f; //< i.e. index straight
        for (int8_t n = 1;; n++) {
          auto i_ff = i_st.next();
          if (!canGo(Vector(i_ff), i_ff.getDir()))
            break;
          callback(i_st, getEdgeCost(ST_DIAG, n));
          i_st = i_ff;
        }
        /* 左右を一般化 */
        auto nd_r45 = arrow_diag_to_along_45();
        auto d_45 = nd + nd_r45;
        auto nd_90 = nd + nd_r45 * 2;
        auto d_135 = nd + nd_r45 * 3;
        auto v_45 = i_f.arrow_to();
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
  };
  static_assert(sizeof(Index) == 2, "Index Size Error"); /**< Size Check */
  typedef std::vector<Index> Indexs;
  /**
   * @brief Graph の Node
   */
  struct __attribute__((__packed__)) Node {
    cost_t cost;
    Index from;
    Node(const cost_t cost = CostMax) : cost(cost) {}
  };
  static_assert(sizeof(Node) == 4, "Node Size Error"); /**< Size Check */

  /**
   * @brief Get the Huristic Value
   * @param i Index
   * @return cost_t huristic value
   */
  cost_t getHuristic(const Index i) const {
    // return 0;
    const auto v = Vector(i) - maze.getGoals()[0];
    const float x = std::abs(v.x);
    const float y = std::abs(v.y);
    const float d = x + y;
    // const float d = std::sqrt(v.x * v.x + v.y * v.y);
    // const float d = std::max(x, y);
    // const float d = v.x * v.x + v.y * v.y;
    return getEdgeCost(ST_DIAG, d);
  }
  bool calcShortestPath(Indexs &path, bool known_only = true,
                        bool diag_enabled = true) {
    std::unordered_map<Index, Node, Index::hash> node_map;
    std::function<bool(const Index &i1, const Index &i2)> greater =
        [&](const auto &i1, const auto &i2) {
          return node_map[i1].cost > node_map[i2].cost;
        };
    std::priority_queue<Index, std::vector<Index>, decltype(greater)> open_list(
        greater);
    const auto start_index = Index(0, 0, Dir::AbsMax, Dir::North);
    node_map[start_index] = Node(0);
    open_list.push(start_index);
    Index goal_index; //< 終点の用意
    while (1) {
      if (open_list.empty()) {
        std::cerr << "open_list.empty()" << std::endl;
        return false;
      }
      const auto index = open_list.top();
      open_list.pop();
      // std::cout << "top:\t" << index << "\t: " << node_map[index].cost
      //           << std::endl; //< print
      const auto &goals = maze.getGoals();
      if (index.getNodeDir().isAlong() &&
          goals.cend() !=
              std::find_if(goals.cbegin(), goals.cend(),
                           [&](const auto v) { return v == Vector(index); })) {
        goal_index = index;
        break;
      }
      index.neighbors_for(maze, known_only, diag_enabled,
                          [&](const auto nibr_index, const auto edge_cost) {
                            Node &neighbor_node = node_map[nibr_index];
                            cost_t h_n = getHuristic(index);
                            cost_t h_m = getHuristic(nibr_index);
                            cost_t g_n = node_map[index].cost - h_n;
                            cost_t f_m_prime = g_n + edge_cost + h_m;
                            if (f_m_prime < neighbor_node.cost) {
                              neighbor_node.cost = f_m_prime;
                              neighbor_node.from = index;
                              open_list.push(nibr_index);
                            }
                            // std::cout << "  - \t" << nibr_index << "\t: "
                            // << neighbor_node.cost
                            //           << std::endl; //< print
                          });
    }
    path.erase(path.begin(), path.end());
    for (auto i = goal_index; i != start_index; i = node_map[i].from) {
      path.push_back(i);
      // std::cout << i << std::endl;
    }
    path.push_back(start_index);
    // std::cout << start_index << std::endl;
    std::reverse(path.begin(), path.end());
    return true;
  }
  void printPath(std::ostream &os, const Indexs indexs) const {
    int steps[MAZE_SIZE][MAZE_SIZE] = {0};
    int counter = 1;
    for (const auto i : indexs) {
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
  static const Dirs indexs2dirs(const Indexs &path, const bool diag_enabled) {
    if (!diag_enabled) {
      Dirs dirs;
      for (int i = 0; i < (int)path.size() - 1; ++i) {
        const auto nd = path[i].getNodeDir();
        const auto v = Vector(path[i + 1]) - Vector(path[i]);
        for (int j = 0; j < std::abs(v.x) + std::abs(v.y); ++j)
          dirs.push_back(nd);
      }
      dirs.push_back(path.back().getNodeDir());
      return dirs;
    }
    Dirs dirs;
    for (int i = 0; i < (int)path.size() - 1; ++i) {
      auto nd = path[i].getNodeDir();
      auto rel_v = Vector(path[i + 1]) - Vector(path[i]);
      auto rel_nd = Dir(path[i + 1].getNodeDir() - path[i].getNodeDir());
      switch (rel_nd) {
      case Dir::Front:
        if (nd.isAlong()) {
          for (int j = 0; j < std::abs(rel_v.x) + std::abs(rel_v.y); ++j)
            dirs.push_back(nd);
        } else {
          for (auto index = path[i]; index != path[i + 1];
               index = index.next()) {
            const auto nd_45 = index.arrow_diag_to_along_45();
            dirs.push_back(index.getNodeDir() + nd_45);
          }
        }
        break;
      case Dir::Left45:
        if (nd.isAlong()) {
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Left);
        } else {
          dirs.push_back(nd + Dir::Left45);
        }
        break;
      case Dir::Right45:
        if (nd.isAlong()) {
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Right);
        } else {
          dirs.push_back(nd + Dir::Right45);
        }
        break;
      case Dir::Left:
        if (nd.isAlong()) {
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Left);
        } else {
          /* V90 */
          dirs.push_back(nd + Dir::Left45);
          dirs.push_back(nd + Dir::Left135);
        }
        break;
      case Dir::Right:
        if (nd.isAlong()) {
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Right);
        } else {
          /* V90 */
          dirs.push_back(nd + Dir::Right45);
          dirs.push_back(nd + Dir::Right135);
        }
        break;
      case Dir::Left135:
        if (nd.isAlong()) {
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Left);
          dirs.push_back(nd + Dir::Back);
        } else {
          dirs.push_back(nd + Dir::Left45);
          dirs.push_back(nd + Dir::Left135);
        }
        break;
      case Dir::Right135:
        if (nd.isAlong()) {
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Right);
          dirs.push_back(nd + Dir::Back);
        } else {
          dirs.push_back(nd + Dir::Right45);
          dirs.push_back(nd + Dir::Right135);
        }
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
    }
    return dirs;
  }

private:
  const Maze &maze; /**< @brief 使用する迷路の参照 */
};

} // namespace MazeLib
