#pragma once

#include "Maze.h"
#include <algorithm>
#include <queue>

namespace MazeLib {

class ShortestAlgorithm {
public:
  typedef uint16_t cost_t;
  struct NodeDir {
    enum { East, NorthEast, North, NorthWest };
  };
  union Index {
    struct {
      uint8_t x : 5;
      uint8_t y : 5;
      uint8_t z : 3;
    };
    uint16_t id;
    Index(uint16_t id) : id(id) {}
    Index(uint8_t x, uint8_t y, uint8_t z) : x(x), y(y), z(z) {}
    static constexpr int ZMaz = 6;
    static constexpr int IdMax = MAZE_SIZE * MAZE_SIZE * 6;
    enum Z : uint8_t {
      CH,
      CV,
      ENE,
      ENW,
      NNE,
      NNW,
    };
    operator uint16_t() const { return id; }
    const std::vector<Index> neighbors(const Maze &maze,
                                       const bool known_only) const {
      auto canGo = [&](const int8_t x, const int8_t y, const Dir d) {
        return !maze.isWall(x, y, d) && (known_only && maze.isKnown(x, y, d));
      };
      std::vector<Index> nbrs;
      switch (z) {
      case CH: {
        Dir d = Dir::East;                          //< 仮想方向
        Vector v = Vector(x, y);                    //< 仮想位置
        if (canGo(x, y, d)) {                       //< 進行方向
          v = v.next(d);                            //< 前進
          if (canGo(v.x, v.y, d))                   //< 進行方向前
            nbrs.push_back(Index(v.x, v.y, z));     //< ST
          if (canGo(v.x, v.y, Dir::North)) {        //< 進行方向斜め
            v = v.next(d + Dir::Left);              //< 左折
            if (canGo(v.x, v.y, d))                 //< 進行方向斜め前
              nbrs.push_back(Index(x + 1, y, NNE)); //< L45
            if (canGo(v.x, v.y, d + Dir::Left))     //< 進行方向斜め横
              nbrs.push_back(Index(v.x, v.y, CV));  //< L90
            if (canGo(v.x, v.y, Dir::Back))         //< 進行方向斜め手前
              nbrs.push_back(Index(x, y + 1, ENW)); //< L135
          }
        }
      }
      default:
        break;
      }
      return nbrs;
    }
  };
  struct Node {
    Index from;
    cost_t cost = 0;
    enum State : uint8_t { None, Open, Closed } state = Closed;
  };
  bool calcShortestPath() {
    while (!q.empty())
      q.pop();
    auto goal_indexs = std::vector<Index>{Index(1, 2, Index::CH)};
    auto start_index = Index(0, 0, Index::CV);
    auto &start_node = nodes[start_index.id];
    start_node.state = Node::Open;
    start_node.cost = 0;
    while (1) {
      if (q.empty()) {
        std::cerr << "q.empty()" << std::endl;
        return false;
      }
      const auto index = q.top();
      const auto it =
          std::find(goal_indexs.cbegin(), goal_indexs.cend(), index);
      if (it != goal_indexs.cend()) {
        nodes[index].state = Node::Closed;
        break;
      }
      //   for()
      //   cost_t new_cost =
      q.pop();
    }
    // auto node = q.top();
  }

private:
  Node nodes[Index::IdMax];
  std::priority_queue<cost_t, std::vector<cost_t>, std::greater<cost_t>> q;
};

} // namespace MazeLib
