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
    for (const auto d : {Dir::East, Dir::North})
      for (const auto nd :
           {Dir::NorthEast, Dir::NorthWest, Dir::SouthWest, Dir::SouthEast})
        for (int x = 0; x < MAZE_SIZE; x++)
          for (int y = 0; y < MAZE_SIZE; y++)
            node_map[Index(x, y, d, nd)] = Node();
  }

public:
  typedef float cost_t;
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
    static std::array<cost_t, MAZE_SIZE> cost_table_along;
    static std::array<cost_t, MAZE_SIZE * 2> cost_table_diag;
    static bool initialized = false;
    if (!initialized) {
      initialized = true;
      /* 台形加速のコストテーブルを事前に用意 */
      static const float a = 9000.0f;
      static const float v_s = 300.0f;
      for (int i = 0; i < MAZE_SIZE; ++i) {
        const float x = 90.0f * (i + 1) / 2;
        const float t = (std::sqrt(v_s * v_s + 2 * a * x) - v_s) / a;
        cost_table_along[i] = 2 * t;
      }
      for (int i = 0; i < MAZE_SIZE * 2; ++i) {
        const float x = 1.41421356f * 45.0f * (i + 1) / 2;
        const float t = (std::sqrt(v_s * v_s + 2 * a * x) - v_s) / a;
        cost_table_diag[i] = 2 * t;
      }
    }
    switch (p) {
    case ST_ALONG:
      return cost_table_along[n - 1];
    case ST_DIAG:
      return cost_table_diag[n - 1];
    case F45:
      return 0.323148;
    case F90:
      return 0.445484;
    case F135:
      return 0.464955;
    case F180:
      return 0.592154;
    case FV90:
      return 0.450525;
    case FS90:
      return 0.279919;
    }
    std::cerr << "Unknown Pattern" << std::endl;
    return 1.0e3f;
  }
  union Index {
  private:
    struct {
      uint8_t x;  //< x
      uint8_t y;  //< y
      uint8_t d;  //< position assign
      uint8_t nd; //< dir of node
    };
    uint32_t all;

  public:
    Index(uint8_t x, uint8_t y, Dir d, Dir nd)
        : x(x), y(y), d(d & 7), nd(nd & 7) {
      uniquify();
    }
    Index(Vector v, Dir d, Dir nd) : x(v.x), y(v.y), d(d & 7), nd(nd & 7) {
      uniquify();
    }
    Index(const Index &obj) : all(obj.all) {}
    Index(const uint32_t all = 0) : all(all) { uniquify(); }
    /**
     * @brief for unordered_map
     */
    struct hash {
      size_t operator()(const Index &obj) const { return obj.all; }
    };
    void uniquify() {
      /* 冗長表現の座標を一意にする */
      if (d == Dir::West) {
        d = Dir::East;
        x--;
      }
      if (d == Dir::South) {
        d = Dir::North;
        y--;
      }
    }
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
      switch (nd) {
      /* 区画の中央 */
      case Dir::East:
      case Dir::North:
      case Dir::West:
      case Dir::South: {
        Vector v = Vector(x, y); //< 仮想位置
        /* 直前の壁 */
        if (!canGo(v, nd)) {
          std::cerr << "Something Wrong" << std::endl;
          break;
        }
        /* 直進で行けるところまで行く */
        auto v_st = v.next(nd);
        for (int n = 1;; n++) {
          if (!canGo(v_st, nd))
            break;
          callback(Index(v_st, Dir::AbsMax, nd), getEdgeCost(ST_ALONG, n));
          v_st = v_st.next(nd);
        }
        auto d_f = nd;          //< 前方
        auto v_f = v.next(d_f); //< 前方の区画
        /* 左右を一般化 */
        for (const auto nd_45 : {Dir::Left45, Dir::Right45}) {
          const auto nd_90 = nd_45 * 2;
          const auto nd_135 = nd_45 * 3;
          const auto d_l = d_f + nd_90; //< 左方向
          /* 横壁 */
          if (canGo(v_f, d_l)) {       //< 45度方向の壁
            auto v_fl = v_f.next(d_l); //< 前左の区画
            if (canGo(v_fl, d_f))      //< 45度先の壁
              callback(Index(v_f, d_l, nd + nd_45), getEdgeCost(F45));
            if (canGo(v_fl, d_l)) //< 90度先の壁
              callback(Index(v_fl, Dir::AbsMax, nd + nd_90), getEdgeCost(F90));
            auto d_b = d_f + Dir::Back;    //< 後方向
            if (canGo(v_fl, d_b)) {        //< 135度の壁
              auto v_fll = v_fl.next(d_b); //< 前左左の区画
              if (canGo(v_fll, d_l))       //< 135度行先
                callback(Index(v_fll, d_f, nd + nd_135), getEdgeCost(F135));
              if (canGo(v_fll, d_b)) //< 180度行先の壁
                callback(Index(v_fll, Dir::AbsMax, nd + Dir::Back),
                         getEdgeCost(F180));
            }
          }
        }
      } break;
        /* 壁の中央 */
      default: {
        /* 左右を一般化 */
        auto nd_45 = ((d == Dir::East &&
                       (nd == Dir::NorthEast || nd == Dir::SouthWest)) ||
                      (d == Dir::North &&
                       (nd == Dir::NorthWest || nd == Dir::SouthEast)))
                         ? Dir::Left45
                         : Dir::Right45;
        const auto nd_90 = nd_45 * 2;
        const auto nd_135 = nd_45 * 3;
        const auto i_f = this->next();
        /* 直前の壁 */
        if (!canGo(Vector(i_f), i_f.d)) {
          std::cerr << "Something Wrong" << std::endl;
          break;
        }
        /* 直進で行けるところまで行く */
        auto i_st = i_f;
        for (auto n = 1;; n++) {
          auto i_ff = i_st.next(); //< 前方の前方
          if (!canGo(Vector(i_ff), i_ff.d))
            break;
          callback(i_st, getEdgeCost(ST_DIAG, n)); //< ST_DIAG
          i_st = i_ff;
        }
        /* 45度方向*/
        auto d_45 = nd + nd_45;
        auto v_45 = i_f.arrow_to();
        if (canGo(v_45, d_45))
          callback(Index(v_45, Dir::AbsMax, d_45),
                   getEdgeCost(F45)); //< F45
        /* V90方向, 135度方向*/
        auto d_90 = nd + nd_90;
        auto d_135 = nd + nd_135;
        if (canGo(v_45, d_135)) {
          auto v_135 = v_45.next(d_135); //< 135度方向位置
          if (canGo(v_135, d_45))
            callback(Index(v_45, d_135, d_90),
                     getEdgeCost(FV90)); //< FV90
          if (canGo(v_135, d_135))       //< 135度方向
            callback(Index(v_135, Dir::AbsMax, d_135),
                     getEdgeCost(F135)); //< F135
        }
      } break;
      }
    }
  };
  typedef std::vector<Index> Indexs;
  struct Node {
    Index from;
    cost_t cost = 0;
    enum State : uint8_t { None, Open, Closed } state = None;
  };
  cost_t getHuristic(const Index i, const Indexs &index_goals) const {
    return 0;
    auto v = Vector(i) - Vector(index_goals[0]);
    auto v_ref = 1200.0f;
    // return (std::abs(v.x) + std::abs(v.y)) / v_ref;
    return std::sqrt(v.x * v.x + v.y * v.y) / v_ref;
  }
  bool calcShortestPath(Indexs &path, bool known_only = true) {
    std::function<bool(const Index &i1, const Index &i2)> greater =
        [&](const auto &i1, const auto &i2) {
          return node_map[i1].cost > node_map[i2].cost;
        };
    std::priority_queue<Index, std::vector<Index>, decltype(greater)> q(
        greater);
    while (!q.empty())
      q.pop();
    auto start_index = Index(0, 0, Dir::AbsMax, Dir::North);
    auto &start_node = node_map[start_index];
    Indexs goal_indexs;
    for (const auto v : maze.getGoals()) {
      goal_indexs.push_back(Index(v, Dir::AbsMax, Dir::East));
      goal_indexs.push_back(Index(v, Dir::AbsMax, Dir::North));
      goal_indexs.push_back(Index(v, Dir::AbsMax, Dir::West));
      goal_indexs.push_back(Index(v, Dir::AbsMax, Dir::South));
    }
    Index goal_index; //< 終点の用意
    /* 3. */
    start_node.state = Node::Open;
    start_node.cost = 0;
    q.push(start_index);
    while (1) {
      /* 4. */
      // std::cout << "q.size(): " << q.size() << std::endl;
      if (q.empty()) {
        std::cerr << "q.empty()" << std::endl;
        return false;
      }
      /* 5. */
      const Index index = q.top();
      q.pop();
      std::cout << "top:\t" << index << "\t: " << node_map[index].cost
                << std::endl; //< print
      /* 6. */
      const auto it =
          std::find(goal_indexs.cbegin(), goal_indexs.cend(), index);
      if (it != goal_indexs.cend()) {
        /* GOAL! */
        goal_index = *it;
        break;
      }
      node_map[index].state = Node::Closed;
      /* 7. */
      index.neighbors_for(maze, known_only,
                          [&](const auto neighbor_index, const auto edge_cost) {
                            Node &neighbor_node = node_map[neighbor_index];
                            cost_t h_n = getHuristic(index, goal_indexs);
                            cost_t h_m =
                                getHuristic(neighbor_index, goal_indexs);
                            cost_t g_n = node_map[index].cost - h_n;
                            cost_t f_m_prime = g_n + edge_cost + h_m;
                            switch (neighbor_node.state) {
                            case Node::None:
                              neighbor_node.cost = f_m_prime;
                              neighbor_node.state = Node::Open;
                              neighbor_node.from = index;
                              q.push(neighbor_index);
                              break;
                            case Node::Open:
                            case Node::Closed:
                              if (f_m_prime < neighbor_node.cost) {
                                neighbor_node.cost = f_m_prime;
                                neighbor_node.state = Node::Open;
                                neighbor_node.from = index;
                                q.push(neighbor_index);
                              }
                              break;
                            default:
                              break;
                            }
                            std::cout << "  - \t" << neighbor_index
                                      << "\t: " << neighbor_node.cost
                                      << std::endl; //< print
                          });
    }
    /* GOAL */
    /* 9. */
    for (auto i = goal_index; i != start_index; i = node_map[i].from) {
      path.push_back(i);
      // std::cout << i << std::endl;
    }
    std::reverse(path.begin(), path.end());
    Dirs dirs;
    for (auto i : path) {
      auto nd = i.getNodeDir();
      auto d = i.getDir();
      switch (nd) {
      case Dir::East:
      case Dir::North:
      case Dir::West:
      case Dir::South:
        dirs.push_back(nd);
        break;
      case Dir::NorthEast:
        if (d == Dir::East) {
          dirs.push_back(Dir::East);
          dirs.push_back(Dir::North);
        } else {
          dirs.push_back(Dir::North);
          dirs.push_back(Dir::East);
        }
        break;
      case Dir::NorthWest:
        if (d == Dir::East) {
          dirs.push_back(Dir::West);
          dirs.push_back(Dir::North);
        } else {
          dirs.push_back(Dir::North);
          dirs.push_back(Dir::West);
        }
        break;
      case Dir::SouthWest:
        if (d == Dir::East) {
          dirs.push_back(Dir::West);
          dirs.push_back(Dir::South);
        } else {
          dirs.push_back(Dir::South);
          dirs.push_back(Dir::West);
        }
        break;
      case Dir::SouthEast:
        if (d == Dir::East) {
          dirs.push_back(Dir::East);
          dirs.push_back(Dir::South);
        } else {
          dirs.push_back(Dir::South);
          dirs.push_back(Dir::East);
        }
        break;
      }
    }
    return true;
  }
  void printPath(std::ostream &os, const std::vector<Index> indexs) const {
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

private:
  const Maze &maze;
  std::unordered_map<Index, Node, Index::hash> node_map;
};

} // namespace MazeLib
