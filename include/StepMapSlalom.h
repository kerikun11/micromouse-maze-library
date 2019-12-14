/**
 * @file StepMapSlalom.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief スラロームのコストベースのステップマップを表現するクラス
 * @date 2019-08-17
 */
#pragma once

#include "Maze.h"

#include <algorithm> /*< for std::find_if, etc. */
#include <functional>
#include <iomanip> /*< for std::setw() */
#include <limits>  /*< for std::numeric_limits */
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
        return cost_table_along[n]; /*< [ms] */
      case ST_DIAG:
        return cost_table_diag[n]; /*< [ms] */
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
      const auto d = seg * i; /*< i区画分の走行距離 */
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
  struct __attribute__((__packed__)) Index {
  private:
    int x : 6; /**< @brief x coordinate of the cell */
    int y : 6; /**< @brief y coordinate of the cell */
    unsigned int
        z : 1; /**< @brief position assignment in the cell, 0:East; 1:North */
    unsigned int nd : 3; /**< @brief direction of the node */

  public:
    /**
     * @brief 迷路中の Index の総数．
     */
    static constexpr int SIZE = MAZE_SIZE_MAX * MAZE_SIZE_MAX * 12;

  public:
    /** @brief デフォルトコンストラクタ */
    Index() {}
    /** @brief 成分を受け取ってそのまま代入するコンストラクタ */
    Index(const int8_t x, const int8_t y, const uint8_t z, const Direction nd)
        : x(x), y(y), z(z), nd(nd) {}
    /** @brief 冗長性を除去するコンストラクタ */
    Index(const Position &p, const Direction d, const Direction nd)
        : x(p.x), y(p.y), nd(nd) {
      uniquify(d);
    }
    /** @brief 区画中央のコンストラクタ */
    Index(const int8_t x, const int8_t y, const Direction nd)
        : x(x), y(y), z(0), nd(nd) {}
    Index(const Position &p, const Direction nd)
        : x(p.x), y(p.y), z(0), nd(nd) {}
    /** @brief 壁上のコンストラクタ */
    Index(const WallIndex &i, const Direction nd)
        : x(i.x), y(i.y), z(i.z), nd(nd) {}
    /** @brief 等号 */
    bool operator==(const Index &i) const {
      return x == i.x && y == i.y && z == i.z && nd == i.nd;
    }
    bool operator!=(const Index &i) const {
      return x != i.x || y != i.y || z != i.z || nd != i.nd;
    }
    /** @brief WallIndex へのキャスト */
    operator WallIndex() const {
      const auto nd = getNodeDirection();
      return nd.isAlong() ? WallIndex(Position(x, y), nd) : WallIndex(x, y, z);
    }
    /**
     * @brief 迷路中のIndexをuniqueな通し番号として表現したID
     * @return uint16_t ID
     */
    uint16_t getIndex() const {
      return (((~nd) & 1) << (2 * MAZE_SIZE_BIT + 3)) |
             (z << (2 * MAZE_SIZE_BIT + 2)) |
             ((6 & nd) << (2 * MAZE_SIZE_BIT - 1)) | (x << MAZE_SIZE_BIT) |
             y; /*< M * M * 12 */
    }
    /**
     * @brief 座標の冗長を一意にする．
     * d を East or North のどちらかにそろえる
     */
    void uniquify(const Direction d) {
      z = (d >> 1) & 1; /*< East,West => 0, North,South => 1 */
      if (d == Direction::West)
        x--;
      if (d == Direction::South)
        y--;
    }
    /**
     * @brief Getters
     */
    const Position getPosition() const { return Position(x, y); }
    const Direction getDirection() const {
      return z == 0 ? Direction::East : Direction::North;
    }
    const Direction getNodeDirection() const { return nd; }
    const WallIndex getWallIndex() const {
      const auto nd = getNodeDirection();
      return nd.isAlong() ? WallIndex(Position(x, y), nd) : WallIndex(x, y, z);
    }
    /**
     * @brief stream での表示
     */
    friend std::ostream &operator<<(std::ostream &os, const Index &i);
    /**
     * @brief 斜め方向に向いているときの区画への相対方向(±45度)を返す
     * @return const Direction Direction::Left45 or Direction::Right45
     */
    const Direction arrow_diag_to_along_rel_45() const {
      switch (nd) {
      case Direction::NorthEast:
      case Direction::SouthWest:
        return z == 0 ? Direction::Left45 : Direction::Right45;
      case Direction::NorthWest:
      case Direction::SouthEast:
        return z == 1 ? Direction::Left45 : Direction::Right45;
      default:
        logw << "Invalid Direction: " << nd << std::endl;
        return Direction::Max;
      }
    }
    /**
     * @brief NodeDirection が向いている方向の隣の Index を返す
     * @return const Index 隣接 Index
     */
    const Index next(const Direction nd) const;
    /**
     * @brief 反対向きのIndexを取得
     * @return const Index 反対向き Index
     */
    const Index opposite() const {
      return Index(x, y, z, nd + Direction::Back);
    }
  };
  static_assert(sizeof(Index) == 2, "size error"); /**< size check */
  using Indexes = std::vector<Index>;

public:
  StepMapSlalom() {}
  bool calcShortestDirections(const Maze &maze, const EdgeCost &edge_cost,
                              Directions &shortest_dirs, const bool known_only,
                              const bool diag_enabled);
  void update(const Maze &maze, const EdgeCost &edge_cost, const Indexes &dest,
              const bool known_only, const bool diag_enabled);
  bool genPathFromMap(Indexes &path) const;
  void print(const Maze &maze, const Indexes &indexes,
             std::ostream &os = std::cout) const;
  cost_t getShortestCost() const {
    return cost_map[index_start.opposite().getIndex()];
  }

  static const Indexes convertDestinations(const Positions &src);
  static const Directions indexes2directions(const Indexes &path,
                                             const bool diag_enabled);

private:
  const Index index_start = Index(Position(0, 0), Direction::North);
  std::array<Index, Index::SIZE> from_map;
  std::array<cost_t, Index::SIZE> cost_map;
};

} // namespace MazeLib
