#pragma once

#include "Maze.h"

#include <algorithm>
#include <functional>
#include <iomanip> //< for std::setw()
#include <queue>
#include <unordered_map>

namespace MazeLib {

class ShortestAlgorithm {
public:
  ShortestAlgorithm(const Maze &maze) : maze(maze) {
    /* メモリの確保 */
    // for (const auto d : {Dir::East, Dir::North})
    //   for (const auto nd :
    //        {Dir::NorthEast, Dir::NorthWest, Dir::SouthWest, Dir::SouthEast})
    //     for (int x = 0; x < MAZE_SIZE; x++)
    //       for (int y = 0; y < MAZE_SIZE; y++)
    //         node_map[Index(x, y, d, nd)] = Node();
    /* テーブルの計算 */
    getEdgeCost(ST_ALONG);
  }

public:
  typedef uint16_t cost_t;
  enum Pattern {
    ST_ALONG,
    ST_DIAG,
    F45,
    F90,
    F135,
    F180,
    FV90,
    FS90,
  };
  static cost_t getEdgeCost(const enum Pattern p, const int n = 1) {
    static std::array<cost_t, MAZE_SIZE * 2> cost_table_along;
    static std::array<cost_t, MAZE_SIZE * 2> cost_table_diag;
    static bool initialized = false;
    if (!initialized) {
      initialized = true;
      /* 台形加速のコストテーブルを事前に用意 */
      static const float a = 1200.0f;
      static const float v_s = 240.0f;
      for (int i = 0; i < MAZE_SIZE * 2; ++i) {
        const float x = 90.0f * (i + 1) / 2;
        const float t = (std::sqrt(v_s * v_s + 2 * a * x) - v_s) / a;
        cost_table_along[i] = 2 * t * 1000;
      }
      for (int i = 0; i < MAZE_SIZE * 2; ++i) {
        const float x = 1.41421356f * 45.0f * (i + 1) / 2;
        const float t = (std::sqrt(v_s * v_s + 2 * a * x) - v_s) / a;
        cost_table_diag[i] = 2 * t * 1000;
      }
    }
    switch (p) {
    case ST_ALONG:
      return cost_table_along[n - 1];
    case ST_DIAG:
      return cost_table_diag[n - 1];
    case F45:
      return 323;
    case F90:
      return 445;
    case F135:
      return 464;
    case F180:
      return 592;
    case FV90:
      return 450;
    case FS90:
      return 279;
    }
    std::cerr << "Unknown Pattern" << std::endl;
    return 0;
  }
  /**
   * @brief Graph の Node の Index．
   * 変数表現は隣接区画で冗長になりうるが，コンストラクタで一意にしているので，メンバ変数は一意．
   * 各区画中央の4方位，各壁中央の4方位，の位置姿勢を識別する
   */
  union Index {
  private:
    struct {
      uint8_t x;  //< x
      uint8_t y;  //< y
      uint8_t d;  //< position assignment in a cell
      uint8_t nd; //< real dir of node
    };
    uint32_t all;

  public:
    Index(const uint8_t x, const uint8_t y, const Dir d, const Dir nd)
        : x(x), y(y), d(d & 7), nd(nd & 7) {
      uniquify();
    }
    Index(const Vector v, const Dir d, const Dir nd)
        : x(v.x), y(v.y), d(d & 7), nd(nd & 7) {
      uniquify();
    }
    Index(const uint32_t all = 0) : all(all) { uniquify(); }
    Index(const Index &obj) : all(obj.all) {}
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
    void uniquify() {
      if (d == Dir::West) {
        d = Dir::East;
        x--;
      }
      if (d == Dir::South) {
        d = Dir::North;
        y--;
      }
    }
    /**
     * @brief oprators
     */
    const Index &operator=(const Index &obj) { return all = obj.all, *this; }
    bool operator==(const Index &obj) const { return all == obj.all; }
    bool operator!=(const Index &obj) const { return all != obj.all; }
    operator Vector() const { return Vector(x, y); }
    friend std::ostream &operator<<(std::ostream &os, const Index &i) {
      return os << "( " << std::setw(2) << (int)i.x << ", " << std::setw(2)
                << (int)i.y << ", " << i.getDir().toChar() << ", "
                << i.getNodeDir().toChar() << ")";
    }
    const Dir getDir() const { return d; }
    const Dir getNodeDir() const { return nd; }
    const Vector arrow_from() const {
      switch (nd) {
      case Dir::NorthEast:
        return Vector(x, y);
      case Dir::NorthWest:
        return d == Dir::East ? Vector(x + 1, y) : Vector(x, y);
      case Dir::SouthWest:
        return d == Dir::East ? Vector(x + 1, y) : Vector(x, y + 1);
      case Dir::SouthEast:
        return d == Dir::East ? Vector(x, y) : Vector(x, y + 1);
      default:
        break;
      }
      std::cerr << "Invalid Index" << std::endl;
      return Vector(x, y);
    }
    const Vector arrow_to() const {
      switch (nd) {
      case Dir::NorthEast:
        return d == Dir::East ? Vector(x + 1, y) : Vector(x, y + 1);
      case Dir::NorthWest:
        return d == Dir::East ? Vector(x, y) : Vector(x, y + 1);
      case Dir::SouthWest:
        return Vector(x, y);
      case Dir::SouthEast:
        return d == Dir::East ? Vector(x + 1, y) : Vector(x, y);
      default:
        break;
      }
      std::cerr << "Invalid Index" << std::endl;
      return Vector(x, y);
    }
    const Dir arrow_cell_45() const {
      auto nd_45 =
          ((d == Dir::East && (nd == Dir::NorthEast || nd == Dir::SouthWest)) ||
           (d == Dir::North && (nd == Dir::NorthWest || nd == Dir::SouthEast)))
              ? Dir::Left45
              : Dir::Right45;
      return nd_45;
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
        return d == Dir::East ? Index(Vector(x + 1, y), Dir::North, nd)
                              : Index(Vector(x, y + 1), Dir::East, nd);
      case Dir::NorthWest:
        return d == Dir::East ? Index(Vector(x, y), Dir::North, nd)
                              : Index(Vector(x - 1, y + 1), Dir::East, nd);
      case Dir::SouthWest:
        return d == Dir::East ? Index(Vector(x, y - 1), Dir::North, nd)
                              : Index(Vector(x - 1, y), Dir::East, nd);
      case Dir::SouthEast:
        return d == Dir::East ? Index(Vector(x + 1, y - 1), Dir::North, nd)
                              : Index(Vector(x, y), Dir::East, nd);
      default:
        break;
      }
      std::cerr << "Invalid Index" << std::endl;
      return Index();
    }
    void neighbors_for(
        const Maze &maze, const bool known_only,
        std::function<void(const Index, const cost_t)> callback) const {
      /* known_only を考慮した壁の判定式を用意 */
      auto canGo = [&](const Vector vec, const Dir dir) {
        if (maze.isWall(vec, dir))
          return false;
        if (known_only && !maze.isKnown(vec, dir))
          return false;
        return true;
      };
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
        /* ターン */
        const auto d_f = nd;          //< 前方
        const auto v_f = v.next(d_f); //< 前方の区画
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
        if (!canGo(Vector(i_f), i_f.d)) {
          std::cerr << "Something Wrong" << std::endl;
          return;
        }
        /* 直進で行けるところまで行く */
        auto i_st = i_f; //< i.e. index straight
        for (int8_t n = 1;; n++) {
          auto i_ff = i_st.next();
          if (!canGo(Vector(i_ff), i_ff.d))
            break;
          callback(i_st, getEdgeCost(ST_DIAG, n));
          i_st = i_ff;
        }
        /* 左右を一般化 */
        auto nd_r45 = arrow_cell_45();
        auto d_45 = nd + nd_r45;
        auto nd_90 = nd + nd_r45 * 2;
        auto d_135 = nd + nd_r45 * 3;
        auto v_45 = i_f.arrow_to();
        if (canGo(v_45, d_45))
          callback(Index(v_45, Dir::AbsMax, d_45), getEdgeCost(F45));
        /* V90方向, 135度方向*/
        if (canGo(v_45, d_135)) {
          /* V90方向, 135度方向*/
          auto v_135 = v_45.next(d_135); //< 135度方向位置
          if (canGo(v_135, d_45))
            callback(Index(v_45, d_135, nd_90), getEdgeCost(FV90));
          if (canGo(v_135, d_135)) //< 135度方向
            callback(Index(v_135, Dir::AbsMax, d_135), getEdgeCost(F135));
        }
      }
    }
  };
  typedef std::vector<Index> Indexs;
  struct Node {
    enum State : uint8_t { None, Open, Closed } state;
    cost_t cost;
    Index from;
    Node(const enum State state = None, const cost_t cost = 0,
         const Index from = Index())
        : state(state), cost(cost), from(from) {}
  };
  cost_t getHuristic(const Index i, const Vectors &goals) const {
    // return 0;
    auto v = Vector(i) - goals[0];
    float x = std::abs(v.x);
    float y = std::abs(v.y);
    float d = x + y;
    return getEdgeCost(ST_ALONG, d / 90);
  }
  bool calcShortestPath(Indexs &path, bool known_only = true) {
    /* 1. */
    std::unordered_map<Index, Node, Index::hash> node_map;
    std::function<bool(const Index &i1, const Index &i2)> greater =
        [&](const auto &i1, const auto &i2) {
          return node_map[i1].cost > node_map[i2].cost;
        };
    std::priority_queue<Index, std::vector<Index>, decltype(greater)> open_list(
        greater);
    /* 2. */
    /* 3. */
    const auto start_index = Index(0, 0, Dir::AbsMax, Dir::North);
    node_map[start_index] = Node(Node::Open, 0);
    open_list.push(start_index);
    Index goal_index; //< 終点の用意
    while (1) {
      /* 4. */
      // std::cout << "q.size(): " << q.size() << std::endl;
      if (open_list.empty()) {
        std::cerr << "q.empty()" << std::endl;
        return false;
      }
      /* 5. */
      const auto index = open_list.top();
      open_list.pop();
      // std::cout << "top:\t" << index << "\t: " << node_map[index].cost
      //           << std::endl; //< print
      /* 6. */
      const auto &goals = maze.getGoals();
      const auto it =
          std::find_if(goals.cbegin(), goals.cend(), [&](const auto v) {
            /* 斜めでないかつゴール区画 */
            return (index.getNodeDir() & 1) == 0 && v == Vector(index);
          });
      if (it != goals.cend()) {
        /* GOAL! */
        goal_index = index;
        break;
      }
      node_map[index].state = Node::Closed;
      /* 7. */
      index.neighbors_for(
          maze, known_only, [&](const auto nibr_index, const auto edge_cost) {
            Node &neighbor_node = node_map[nibr_index];
            cost_t h_n = getHuristic(index, goals);
            cost_t h_m = getHuristic(nibr_index, goals);
            cost_t g_n = node_map[index].cost - h_n;
            cost_t f_m_prime = g_n + edge_cost + h_m;
            if (neighbor_node.state == Node::None ||
                f_m_prime < neighbor_node.cost) {
              neighbor_node = Node(Node::Open, f_m_prime, index);
              open_list.push(nibr_index);
            }
            // std::cout << "  - \t" << neighbor_index
            //           << "\t: " << neighbor_node.cost
            //           << std::endl; //< print
          });
    }
    /* 9. */
    path.erase(path.begin(), path.end());
    for (auto i = goal_index; i != start_index; i = node_map[i].from) {
      path.push_back(i);
      // std::cout << i << std::endl;
    }
    path.push_back(start_index);
    /* END A* */
    // std::cout << start_index << std::endl;
    std::reverse(path.begin(), path.end());
    Dirs dirs = indexs2dirs(path);
    // maze.printPath(Vector(0, 0), dirs);
    return true;
  }
  void printPath(std::ostream &os, const Indexs indexs) const {
    int steps[MAZE_SIZE][MAZE_SIZE] = {0};
    int counter = 1;
    for (const auto i : indexs) {
      auto v = Vector(i);
      steps[v.y][v.x] = counter++;
    }
    for (int8_t y = MAZE_SIZE; y >= 0; y--) {
      if (y != MAZE_SIZE) {
        os << '|';
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
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
      for (uint8_t x = 0; x < MAZE_SIZE; x++)
        os << "+"
           << (maze.isKnown(x, y, Dir::South)
                   ? (maze.isWall(x, y, Dir::South) ? "---" : "   ")
                   : (C_RED " . " C_RESET));
      os << "+" << std::endl;
    }
  }
  static const Dirs indexs2dirs(const Indexs &path) {
    Dirs dirs;
    for (int i = 0; i < (int)path.size() - 1; i++) {
      // auto v = Vector(path[i]);
      // auto d = path[i].getDir();
      auto nd = path[i].getNodeDir();
      auto rel_v = Vector(path[i + 1]) - Vector(path[i]);
      // auto rel_d = Dir(path[i + 1].getDir() - path[i].getDir());
      auto rel_nd = Dir(path[i + 1].getNodeDir() - path[i].getNodeDir());
      switch (rel_nd) {
      case Dir::Front:
        if (nd.isAlong()) {
          for (int j = 0; j < std::abs(rel_v.x) + std::abs(rel_v.y); j++)
            dirs.push_back(nd);
        } else {
          for (auto index = path[i]; index != path[i + 1];
               index = index.next()) {
            auto nd_45 = ((index.getDir() == Dir::East &&
                           (index.getNodeDir() == Dir::NorthEast ||
                            index.getNodeDir() == Dir::SouthWest)) ||
                          (index.getDir() == Dir::North &&
                           (index.getNodeDir() == Dir::NorthWest ||
                            index.getNodeDir() == Dir::SouthEast)))
                             ? Dir::Left45
                             : Dir::Right45;
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
  const Maze &maze;
};

} // namespace MazeLib
