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
    //< メモリの確保
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
  static cost_t getEdgeCost(enum Pattern p) {
    switch (p) {
    case ST_ALONG:
      return 90.0f / 600.0f;
    case ST_DIAG:
      return 90.0f / 1.41421356f / 600.0f;
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
      uint8_t x;  //< uint5_t
      uint8_t y;  //< uint5_t
      uint8_t d;  //< Dir Object
      uint8_t nd; //< Dir Object
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
        if (!canGo(v, nd)) {
          std::cerr << "Something Wrong" << std::endl;
          break;
        }
        auto d_f = nd;          //< 前方
        auto v_f = v.next(d_f); //< 前方の区画
        if (canGo(v_f, d_f))    //< 前方の前方
          callback(Index(v_f, Dir::AbsMax, nd),
                   getEdgeCost(ST_ALONG)); //< ST_ALONG
                                           /* 左右を一般化 */
        for (const auto nd_45 : {Dir::Left45, Dir::Right45}) {
          const auto nd_90 = nd_45 * 2;
          const auto nd_135 = nd_45 * 3;
          const auto d_l = d_f + nd_90; //< 左方向
          if (canGo(v_f, d_l)) {        //< 45度方向の壁
            auto v_fl = v_f.next(d_l);  //< 前左の区画
            if (canGo(v_fl, d_f))       //< 45度先の壁
              callback(Index(v_f, d_l, nd + nd_45), getEdgeCost(F45)); //< F45
            if (canGo(v_fl, d_l)) //< 90度先の壁
              callback(Index(v_fl, Dir::AbsMax, nd + nd_90),
                       getEdgeCost(F90));  //< F90
            auto d_b = d_f + Dir::Back;    //< 後方向
            if (canGo(v_fl, d_b)) {        //< 135度の壁
              auto v_fll = v_fl.next(d_b); //< 前左左の区画
              if (canGo(v_fll, d_l))       //< 135度行先
                callback(Index(v_fll, d_f, nd + nd_135),
                         getEdgeCost(F135)); //< F135
              if (canGo(v_fll, d_b))         //< 180度行先の壁
                callback(Index(v_fll, Dir::AbsMax, nd + Dir::Back),
                         getEdgeCost(F180)); //< F180
            }
          }
        }
      } break;
      /* 壁の中央 */
      default: {
        auto nd_45 = ((d == Dir::East &&
                       (nd == Dir::NorthEast || nd == Dir::SouthWest)) ||
                      (d == Dir::North &&
                       (nd == Dir::NorthWest || nd == Dir::SouthEast)))
                         ? Dir::Left45
                         : Dir::Right45;
        const auto nd_90 = nd_45 * 2;
        const auto nd_135 = nd_45 * 3;
        const auto i_f = this->next();
        if (canGo(Vector(i_f), i_f.d)) {          //< 前方
          auto i_ff = i_f.next();                 //< 前方の前方
          if (canGo(Vector(i_ff), i_ff.d))        //< 前方の前方
            callback(i_ff, getEdgeCost(ST_DIAG)); //< ST_DIAG
          auto d_45 = nd + nd_45;
          auto v_45 = i_f.arrow_to();
          if (canGo(v_45, d_45))
            callback(Index(v_45, Dir::AbsMax, d_45),
                     getEdgeCost(F45)); //< F45
          auto d_90 = nd + nd_90;
          auto d_135 = nd + nd_135;
          if (canGo(v_45, d_135))
            callback(Index(v_45, d_135, d_90),
                     getEdgeCost(FV90)); //< FV90
          auto v_135 = v_45.next(d_135); //< 135度方向位置
          if (canGo(v_135, d_135))       //< 135度方向
            callback(Index(v_135, Dir::AbsMax, d_135),
                     getEdgeCost(F135)); //< F135
        }
      } break;
      }
    }
  };
  struct Node {
    Index from;
    cost_t cost = 0;
    enum State : uint8_t { None, Open, Closed } state = None;
  };
  bool calcShortestPath(bool known_only = true) {
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
    auto goal_indexs = std::vector<Index>{Index(2, 3, Dir::AbsMax, Dir::West)};
    Index goal_index; //< 終点の用意
    /* 3. */
    start_node.state = Node::Open;
    start_node.cost = 0;
    q.push(start_index);
    while (1) {
      /* 4. */
      std::cout << "q.size(): " << q.size() << std::endl;
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
                            std::cout << "  - \t" << index
                                      << "\t: " << node_map[index].cost
                                      << std::endl; //< print
                            Node &neighbor_node = node_map[neighbor_index];
                            cost_t h_m = 0; //< in progress
                            cost_t h_n = 0; //< in progress
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
                          });
    }
    /* GOAL */
    /* 9. */
    for (auto i = goal_index; i != start_index; i = node_map[i].from) {
      std::cout << i << std::endl;
    }
    return true;
  }

private:
  const Maze &maze;
  std::unordered_map<Index, Node, Index::hash> node_map;
};

} // namespace MazeLib
