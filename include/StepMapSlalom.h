/**
 * @file StepMapSlalom.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief スラロームのコストベースのステップマップを表現するクラス
 * @date 2019-08-17
 */
#pragma once

#include "Maze.h"

#include <algorithm> /*< for std::find_if, etc. */
#include <iomanip>   /*< for std::setw() */
#include <queue>

namespace MazeLib {

/**
 * @brief スラロームのコストベースのステップマップを表現するクラス
 */
class StepMapSlalom {
public:
  using cost_t = uint16_t; /**< @brief 時間コストの型 [ms] */
  static constexpr cost_t CostMax = std::numeric_limits<cost_t>::max();
  enum Pattern : int8_t { ST_ALONG, ST_DIAG, F45, F90, F135, F180, FV90, FS90 };
  /**
   * @brief エッジコストの管理
   */
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
    EdgeCost(const RunParameter rp = RunParameter()) : rp(rp) {
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
        std::cerr << "Unknown Pattern" << std::endl;
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

    static cost_t gen_cost_impl(const int i, const float am, const float vs,
                                const float vm, const float seg) {
      const auto d = seg * (i + 1); /*< (i+1) 区画分の走行距離 */
      /* グラフの面積から時間を求める */
      const auto d_thr = (vm * vm - vs * vs) / am; /*< 最大速度に達する距離 */
      if (d < d_thr)
        return 2 * (std::sqrt(vs * vs + am * d) - vs) / am *
               1000; /*< 三角加速 */
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
    static constexpr int SIZE = MAZE_SIZE * MAZE_SIZE * 12;

  public:
    /**
     * @brief Construct a new Index object
     */
    Index(const int8_t x, const int8_t y, const uint8_t z, const Dir nd)
        : x(x), y(y), z(z), nd(nd) {}
    Index(const int8_t x, const int8_t y, const Dir d, const Dir nd)
        : x(x), y(y), nd(nd) {
      uniquify(d);
    }
    Index(const Vector v, const Dir d, const Dir nd) : x(v.x), y(v.y), nd(nd) {
      uniquify(d);
    }
    Index(const int8_t x, const int8_t y, const Dir nd)
        : x(x), y(y), z(0), nd(nd) {}
    Index(const WallIndex i, const Dir nd) : x(i.x), y(i.y), z(i.z), nd(nd) {}
    Index(const Vector v, const Dir nd) : x(v.x), y(v.y), z(0), nd(nd) {}
    Index() : all(0) {}
    operator WallIndex() const {
      const auto nd = getNodeDir();
      return nd.isAlong() ? WallIndex(x, y, nd) : WallIndex(x, y, z);
    }
    /**
     * @brief unique な ID を返す
     */
    operator uint16_t() const {
      return (((~nd) & 1) << (2 * MAZE_SIZE_BIT + 3)) |
             (z << (2 * MAZE_SIZE_BIT + 2)) |
             ((6 & nd) << (2 * MAZE_SIZE_BIT - 1)) | (x << MAZE_SIZE_BIT) |
             y; /*< M * M * 12 */
    }
    /**
     * @brief 座標の冗長を一意にする．
     * d を East or North のどちらかにそろえる
     */
    void uniquify(const Dir d) {
      z = (d >> 1) & 1; /*< East,West => 0, North,South => 1 */
      if (d == Dir::West)
        x--;
      if (d == Dir::South)
        y--;
    }
    /**
     * @brief Getters
     */
    const Dir getDir() const { return z == 0 ? Dir::East : Dir::North; }
    const Dir getNodeDir() const { return nd; }
    const Vector getVector() const { return Vector(x, y); }
    friend std::ostream &operator<<(std::ostream &os, const Index i) {
      if (i.getNodeDir().isAlong())
        return os << "( " << std::setw(2) << (int)i.x << ", " << std::setw(2)
                  << (int)i.y << ", " << i.getNodeDir().toChar() << ")";
      return os << "( " << std::setw(2) << (int)i.x << ", " << std::setw(2)
                << (int)i.y << ", " << i.getDir().toChar() << ", "
                << i.getNodeDir().toChar() << ")";
    }
    /**
     * @brief 斜め方向に向いているときの区画への相対方向(±45度)を返す
     * @return const Dir Dir::Left45 or Dir::Right45
     */
    const Dir arrow_diag_to_along_rel_45() const {
      switch (nd) {
      case Dir::NorthEast:
      case Dir::SouthWest:
        return z == 0 ? Dir::Left45 : Dir::Right45;
      case Dir::NorthWest:
      case Dir::SouthEast:
        return z == 1 ? Dir::Left45 : Dir::Right45;
      default:
        logw << "Invalid Dir: " << nd << std::endl;
        return Dir::Max;
      }
    }
    /**
     * @brief NodeDir が向いている方向の隣の Index を返す
     * @return const Index
     */
    const Index next(const Dir nd) const {
      switch (getNodeDir()) {
      case Dir::East:
      case Dir::North:
      case Dir::West:
      case Dir::South:
        return nd.isAlong() ? Index(getVector().next(nd), nd)
                            : Index(WallIndex(x, y, getNodeDir()).next(nd), nd);
      case Dir::NorthEast:
        switch (nd) {
        case Dir::East:
          return Index(x + 1, y + 1, nd);
        case Dir::North:
          return Index(x + 1, y + 1, nd);
        case Dir::West:
          return Index(x, y + 1, nd);
        case Dir::South:
          return Index(x + 1, y, nd);
        default:
          return Index(WallIndex(x, y, z).next(nd), nd);
        }
      case Dir::NorthWest:
        switch (nd) {
        case Dir::East:
          return Index(x + 1, y + 1, nd);
        case Dir::North:
          return Index(x, y + 1, nd);
        case Dir::West:
          return Index(x - 1, y + 1, nd);
        case Dir::South:
          return Index(x - 1, y, nd);
        default:
          return Index(WallIndex(x, y, z).next(nd), nd);
        }
      case Dir::SouthWest:
        switch (nd) {
        case Dir::East:
          return Index(x + 1, y - 1, nd);
        case Dir::North:
          return Index(x - 1, y + 1, nd);
        case Dir::West:
          return Index(x - 1, y, nd);
        case Dir::South:
          return Index(x, y - 1, nd);
        default:
          return Index(WallIndex(x, y, z).next(nd), nd);
        }
      case Dir::SouthEast:
        switch (nd) {
        case Dir::East:
          return Index(x + 1, y, nd);
        case Dir::North:
          return Index(x + 1, y + 1, nd);
        case Dir::West:
          return Index(x, y - 1, nd);
        case Dir::South:
          return Index(x + 1, y - 1, nd);
        default:
          return Index(WallIndex(x, y, z).next(nd), nd);
        }
      default:
        logw << "Invalid Dir: " << nd << std::endl;
        return Index(x, y, z, nd);
      }
    }
    const Index opposite() const { return Index(x, y, z, nd + Dir::Back); }
  };
  static_assert(sizeof(Index) == 2, "size error"); /**< size check */
  using Indexes = std::vector<Index>;

public:
  StepMapSlalom() {}
  bool calcShortestDirs(const Maze &maze, const EdgeCost &edge_cost,
                        Dirs &shortest_dirs, const bool known_only,
                        const bool diag_enabled) {
    const auto dest = convertDestinations(maze.getGoals());
    update(maze, edge_cost, dest, known_only, diag_enabled);
    StepMapSlalom::Indexes path;
    if (!genPathFromMap(path))
      return false;
    shortest_dirs = indexes2dirs(path, diag_enabled);
    return true;
  }
  void update(const Maze &maze, const EdgeCost &edge_cost, const Indexes &dest,
              const bool known_only, const bool diag_enabled) {
    /* 全ノードのコストを最大値に設定 */
    for (auto &f : cost_map)
      f = CostMax;
      /* 更新予約のキュー */
#define STEP_MAP_USE_PRIORITY_QUEUE 0
#if STEP_MAP_USE_PRIORITY_QUEUE == 1
    std::function<bool(const Index, const Index)> greater =
        [&](const Index i1, const Index i2) {
          return cost_map[i1] > cost_map[i2];
        };
    std::priority_queue<Index, std::vector<Index>, decltype(greater)> q(
        greater);
#else
    std::queue<Index> q;
#endif
    /* dest のコストを0とする */
    for (const auto i : dest) {
      cost_map[i] = 0;
      q.push(i);
    }
    /* known_only を考慮した壁の判定式を用意 */
    const auto canGo = [&](const WallIndex i) {
      if (maze.isWall(i) || (known_only && !maze.isKnown(i)))
        return false;
      return true;
    };
    /* 更新がなくなるまで更新 */
    while (!q.empty()) {
#if STEP_MAP_USE_PRIORITY_QUEUE
      const auto focus = q.top();
#else
      const auto focus = q.front();
#endif
      q.pop();
      const auto focus_cost = cost_map[focus];
      /* キューに追加する関数を用意 */
      const auto pushAndContinue = [&](const Index next,
                                       const cost_t edge_cost) {
        const auto next_cost = focus_cost + edge_cost;
        if (cost_map[next] <= next_cost)
          return false;
        cost_map[next] = next_cost;
        from_map[next] = focus;
        q.push(next);
        return true;
      };
      const auto nd = focus.getNodeDir();
      if (nd.isAlong()) { /* 区画の中央 */
        /* 直前の壁 */
        if (!canGo(focus))
          continue;
        /* 直進で行けるところまで行く */
        int8_t n = 1;
        for (auto i = focus; canGo(i); ++n) {
          const auto next = i.next(nd);
          if (!pushAndContinue(next, edge_cost.getEdgeCost(ST_ALONG, n)))
            break;
          i = next;
        }
        if (diag_enabled) {
          /* ターン */
          for (const auto nd_rel_45 : {Dir::Left45, Dir::Right45}) {
            const auto d45 = nd + nd_rel_45;
            const auto d90 = nd + nd_rel_45 * 2;
            const auto d135 = nd + nd_rel_45 * 3;
            const auto d180 = nd + nd_rel_45 * 4;
            /* 横壁 */
            const auto i45 = focus.next(d45);
            if (canGo(i45)) {
              /* 45 */
              if (canGo(i45.next(i45.getNodeDir())))
                pushAndContinue(i45, edge_cost.getEdgeCost(F45));
              /* 90 */
              const auto v90 = focus.getVector().next(nd).next(d90);
              pushAndContinue(Index(v90, d90), edge_cost.getEdgeCost(F90));
              /* 135 and 180 */
              const auto i135 = i45.next(d135);
              if (canGo(i135)) {
                /* 135 */
                if (canGo(i135.next(i135.getNodeDir())))
                  pushAndContinue(i135, edge_cost.getEdgeCost(F135));
                /* 180 */
                pushAndContinue(Index(v90.next(d180), d180),
                                edge_cost.getEdgeCost(F180));
              }
            }
          }
        } else {
          /* 斜めなしのターン */
          const auto v_f = focus.getVector().next(nd); //< i.e. vector front
          for (const auto d90 : {nd + Dir::Left, nd + Dir::Right})
            if (canGo(WallIndex(v_f, d90))) //< 90度方向の壁
              pushAndContinue(Index(v_f, d90), edge_cost.getEdgeCost(FS90));
        }
      } else { /* 壁の中央（斜めありの場合しかありえない） */
        /* 直前の壁 */
        const auto i_f = focus.next(nd);
        if (!canGo(i_f)) {
          loge << "FWE: " << focus << std::endl;
          continue;
        }
        /* 直進で行けるところまで行く */
        int8_t n = 1;
        for (auto i = i_f;; ++n) {
          const auto next = i.next(nd);
          if (!canGo(next))
            break;
          if (!pushAndContinue(i, edge_cost.getEdgeCost(ST_ALONG, n)))
            break;
          i = next;
        }
        /* ターン */
        auto nd_r45 = focus.arrow_diag_to_along_rel_45();
        auto d45 = nd + nd_r45;
        auto d90 = nd + nd_r45 * 2;
        auto d135 = nd + nd_r45 * 3;
        /* 45R */
        pushAndContinue(focus.next(d45), edge_cost.getEdgeCost(F45));
        /* V90, 135R */
        const auto i90 = i_f.next(d90);
        if (canGo(i90)) {
          /* V90 */
          if (canGo(i90.next(i90.getNodeDir())))
            pushAndContinue(i90, edge_cost.getEdgeCost(FV90));
          /* 135 R */
          pushAndContinue(focus.next(d135), edge_cost.getEdgeCost(F135));
        }
      }
    }
  }
  bool genPathFromMap(Indexes &path) const {
    path.clear();
    auto i = index_start.opposite();
    while (1) {
      path.push_back(i.opposite());
      if (cost_map[i] == 0)
        break;
      if (cost_map[i] <= cost_map[from_map[i]])
        return false;
      i = from_map[i];
    }
    return true;
  }
  void print(const Maze &maze, const Indexes &indexes,
             std::ostream &os = std::cout) const {
    const auto exists = [&](const Index i) {
      return std::find_if(
                 indexes.cbegin(), indexes.cend(), [&](const Index ii) {
                   if (i.getNodeDir().isAlong() != ii.getNodeDir().isAlong())
                     return false;
                   if (i.getNodeDir().isDiag())
                     return WallIndex(i) == WallIndex(ii);
                   return i.getVector() == ii.getVector();
                 }) != indexes.cend();
    };
    for (int8_t y = MAZE_SIZE - 1; y >= -1; --y) {
      for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
        os << "+";
        if (exists(Index(x, y, 1, Dir::NorthEast)))
          os << C_YE << " X " << C_NO;
        else
          os << (maze.isKnown(x, y, Dir::North)
                     ? (maze.isWall(x, y, Dir::North) ? "---" : "   ")
                     : (C_RE " . " C_NO));
      }
      os << "+" << std::endl;
      if (y != -1) {
        os << '|';
        for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
          if (exists(Index(x, y, 0, Dir::East)))
            os << C_YE << " X " << C_NO;
          else
            os << "   ";
          if (exists(Index(x, y, 0, Dir::NorthEast)))
            os << C_YE << "X" << C_NO;
          else
            os << (maze.isKnown(x, y, Dir::East)
                       ? (maze.isWall(x, y, Dir::East) ? "|" : " ")
                       : (C_RE "." C_NO));
        }
        os << std::endl;
      }
    }
  }
  static const Indexes convertDestinations(const Vectors &src) {
    Indexes dest;
    for (const auto v : src)
      for (const auto nd : Dir::getAlong4())
        dest.push_back(Index(v, nd));
    return dest;
  }
  static const Dirs indexes2dirs(const Indexes &path, const bool diag_enabled) {
    if (!diag_enabled) {
      Dirs dirs;
      for (int i = 1; i < (int)path.size(); ++i) {
        const auto nd = path[i].getNodeDir();
        const auto v = path[i - 1].getVector() - path[i].getVector();
        for (int j = 0; j < std::abs(v.x) + std::abs(v.y); ++j)
          dirs.push_back(nd);
      }
      return dirs;
    }
    Dirs dirs;
    for (int i = 0; i < (int)path.size() - 1; ++i) {
      const auto nd = path[i].getNodeDir();
      const auto rel_v = path[i + 1].getVector() - path[i].getVector();
      const auto rel_nd = Dir(path[i + 1].getNodeDir() - path[i].getNodeDir());
      if (nd.isAlong()) {
        switch (rel_nd) {
        case Dir::Front:
          for (int j = 0; j < std::abs(rel_v.x) + std::abs(rel_v.y); ++j)
            dirs.push_back(nd);
          break;
        case Dir::Left45:
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Left);
          break;
        case Dir::Right45:
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Right);
          break;
        case Dir::Left:
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Left);
          break;
        case Dir::Right:
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Right);
          break;
        case Dir::Left135:
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Left);
          dirs.push_back(nd + Dir::Back);
          break;
        case Dir::Right135:
          dirs.push_back(nd);
          dirs.push_back(nd + Dir::Right);
          dirs.push_back(nd + Dir::Back);
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
        default:
          logw << "invalid Dir" << std::endl;
          break;
        }
      } else {
        switch (rel_nd) {
        case Dir::Front:
          for (auto index = path[i]; index != path[i + 1];
               index = index.next(index.getNodeDir())) {
            const auto nd_45 = index.arrow_diag_to_along_rel_45();
            dirs.push_back(index.getNodeDir() + nd_45);
          }
          break;
        case Dir::Left45:
          dirs.push_back(nd + Dir::Left45);
          break;
        case Dir::Right45:
          dirs.push_back(nd + Dir::Right45);
          break;
        case Dir::Left:
          /* V90 */
          dirs.push_back(nd + Dir::Left45);
          dirs.push_back(nd + Dir::Left135);
          break;
        case Dir::Right:
          /* V90 */
          dirs.push_back(nd + Dir::Right45);
          dirs.push_back(nd + Dir::Right135);
          break;
        case Dir::Left135:
          dirs.push_back(nd + Dir::Left45);
          dirs.push_back(nd + Dir::Left135);
          break;
        case Dir::Right135:
          dirs.push_back(nd + Dir::Right45);
          dirs.push_back(nd + Dir::Right135);
          break;
        default:
          logw << "invalid Dir" << std::endl;
          break;
        }
      }
    }
    return dirs;
  }
  cost_t getShortestCost() const { return cost_map[index_start.opposite()]; }

private:
  const Index index_start = Index(Vector(0, 0), Dir::North);
  std::array<Index, Index::SIZE> from_map;
  std::array<cost_t, Index::SIZE> cost_map;
};

} // namespace MazeLib
