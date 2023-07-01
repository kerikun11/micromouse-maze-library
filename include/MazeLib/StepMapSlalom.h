/**
 * @file StepMapSlalom.h
 * @brief スラロームのコストベースのステップマップを表現するクラス
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-08-17
 * @copyright Copyright 2019 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <limits>  //< for std::numeric_limits
#include <vector>

#include "MazeLib/Maze.h"

namespace MazeLib {

/**
 * @brief スラロームのコストベースのステップマップを表現するクラス
 */
class StepMapSlalom {
 public:
  using cost_t = uint16_t; /**< @brief 時間コストの型 [ms] */
  /**
   * @brief コストの最大値
   */
  static constexpr cost_t CostMax = std::numeric_limits<cost_t>::max();
  /**
   * @brief 最短走行のスラロームのインデックス
   */
  enum Slalom : int8_t { F45, F90, F135, F180, FV90, FS90, FMAX };
  /**
   * @brief エッジコストの管理
   */
  class EdgeCost {
   public:
    /**
     * @brief 走行パラメータの構造体
     * @attention コストの合計が 65,535 [ms] を超えないように注意。
     */
    struct RunParameter {
      RunParameter() {}
      static constexpr float factor = 2;       //< CostMax の超過防止
      float vs = 420.0f * factor;              //< 基本速度 [mm/s]
      float am_a = 4200.0f * factor * factor;  //< 最大加速度 [mm/s/s]
      float am_d = 3600.0f * factor * factor;  //< 最大加速度(斜め) [mm/s/s]
      float vm_a = 1500.0f * factor;           //< 飽和速度 [mm/s]
      float vm_d = 1200.0f * factor;           //< 飽和速度(斜め) [mm/s]
      std::array<cost_t, Slalom::FMAX> slalomCostTable = {{
          cost_t(257 / factor),  //< F45  [ms] @ 412 [mm/s]
          cost_t(375 / factor),  //< F90  [ms] @ 422 [mm/s]
          cost_t(465 / factor),  //< F135 [ms] @ 354 [mm/s]
          cost_t(563 / factor),  //< F180 [ms] @ 412 [mm/s]
          cost_t(388 / factor),  //< FV90 [ms] @ 290 [mm/s]
          cost_t(287 / factor),  //< FS90 [ms] @ 266 [mm/s]
      }};
    };

   public:
    EdgeCost(const RunParameter& rp = RunParameter()) : rp(rp) {
      calcStraightCostTable();
    }
    cost_t getEdgeCostAlong(const int n) const {
      return costTableAlong[n];  //< [ms]
    }
    cost_t getEdgeCostDiag(const int n) const {
      return costTableDiag[n];  //< [ms]
    }
    cost_t getEdgeCostSlalom(const Slalom p) const {
      return rp.slalomCostTable[p];  //< [ms]
    }
    const RunParameter& getRunParameter() const { return rp; }
    void setRunParameter(const RunParameter& rp) {
      this->rp = rp;
      calcStraightCostTable();
    }

   private:
    RunParameter rp; /**< @brief 走行パラメータ */
    /** @brief 台形加速を考慮したコストテーブル (壁沿い) */
    std::array<cost_t, MAZE_SIZE * 2> costTableAlong;
    /** @brief 台形加速を考慮したコストテーブル (斜め) */
    std::array<cost_t, MAZE_SIZE * 2> costTableDiag;

    /**
     * @brief 台形加速を考慮したコストを生成する関数
     * @param i マスの数
     * @param am 最大加速度
     * @param vs 始点速度
     * @param vm 飽和速度
     * @param seg 1マスの長さ
     * @return StepMap::step_t コスト
     */
    static cost_t calcStraightCost(const int i, const float am, const float vs,
                                   const float vm, const float seg) {
      const auto d = seg * i;  //< i 区画分の走行距離
      /* グラフの面積から時間を求める */
      const auto d_thr = (vm * vm - vs * vs) / am;  //< 最大速度に達する距離
      if (d < d_thr)
        return 2 * (std::sqrt(vs * vs + am * d) - vs) / am * 1000;  //< 三角加速
      else
        return (am * d + (vm - vs) * (vm - vs)) / (am * vm) *
               1000;  //< 台形加速
    }
    /**
     * @brief コストテーブルを生成する関数
     * @details 各マス数ごとのコストは不変なので高速化のために予め計算しておく
     */
    void calcStraightCostTable() {
      const float seg_a = 90.0f;
      const float seg_d = 45.0f * std::sqrt(2.0f);
      for (int i = 0; i < MAZE_SIZE * 2; ++i) {
        costTableAlong[i] = calcStraightCost(i, rp.am_a, rp.vs, rp.vm_a, seg_a);
        costTableDiag[i] = calcStraightCost(i, rp.am_d, rp.vs, rp.vm_d, seg_d);
      }
    }
  };

  /**
   * @brief スラローム走行のノードの Index。
   * @details 「各区画中央の4方位」または「
   * 各壁上の4方位」、の位置姿勢を一意に識別する。
   */
  class Index {
   private:
    union {
      struct {
        /** @brief x coordinate of the cell */
        int x : 6;
        /** @brief y coordinate of the cell */
        int y : 6;
        /** @brief position assignment in the cell, 0:East; 1:North */
        unsigned int z : 1;
        /** @brief direction of the node */
        unsigned int nd : 3;
      } __attribute__((__packed__));
      uint16_t data; /**< @brief for access to the entire data */
    };
    static_assert(MAZE_SIZE < std::pow(2, 6), "MAZE_SIZE is too large!");

   public:
    /**
     * @brief 迷路中の Index の総数。for文などに使える。
     * @details x * y * (z と nd の表現数)
     */
    static constexpr int SIZE = MAZE_SIZE_MAX * MAZE_SIZE_MAX * 12;

   public:
    /** @brief デフォルトコンストラクタ */
    Index() {}
    /** @brief 成分を受け取ってそのまま代入するコンストラクタ */
    Index(const int8_t x, const int8_t y, const uint8_t z, const Direction nd)
        : x(x), y(y), z(z), nd(nd) {}
    /** @brief 冗長性を除去するコンストラクタ */
    Index(const Position p, const Direction d, const Direction nd)
        : x(p.x), y(p.y), nd(nd) {
      uniquify(d);
    }
    /** @brief 区画中央のコンストラクタ */
    Index(const int8_t x, const int8_t y, const Direction nd)
        : x(x), y(y), z(0), nd(nd) {}
    /** @brief 区画中央のコンストラクタ */
    Index(const Position p, const Direction nd)
        : x(p.x), y(p.y), z(0), nd(nd) {}
    /** @brief 壁上のコンストラクタ */
    Index(const WallIndex i, const Direction nd)
        : x(i.x), y(i.y), z(i.z), nd(nd) {}
    /** @brief 等号 */
    bool operator==(const Index i) const {
      // return x == i.x && y == i.y && z == i.z && nd == i.nd;
      return data == i.data;  //< 高速化
    }
    /** @brief 等号否定 */
    bool operator!=(const Index i) const {
      // return x != i.x || y != i.y || z != i.z || nd != i.nd;
      return data != i.data;  //< 高速化
    }
    /** @brief WallIndex へのキャスト */
    operator WallIndex() const {
      const auto nd = getNodeDirection();
      return nd.isAlong() ? WallIndex(Position(x, y), nd) : WallIndex(x, y, z);
    }
    /**
     * @brief 迷路中の Index をuniqueな通し番号として表現したID
     * @details 取りうる値を 0-11 に収めるために、bit shift を行っている
     * @return uint16_t ID
     */
    uint16_t getIndex() const {
      return (((~nd) & 1) << (2 * MAZE_SIZE_BIT + 3)) |
             (z << (2 * MAZE_SIZE_BIT + 2)) |
             ((6 & nd) << (2 * MAZE_SIZE_BIT - 1)) | (x << MAZE_SIZE_BIT) |
             y;  //< M * M * 12
    }
    /**
     * @brief 座標の冗長を一意にする。
     * @details d を East or North のどちらかにそろえる
     */
    void uniquify(const Direction d) {
      z = (d >> 1) & 1;  //< East,West => 0, North,South => 1
      switch (d) {
        case Direction::West:
          x--;
          break;
        case Direction::South:
          y--;
          break;
      }
    }
    /** @brief 区画 */
    Position getPosition() const { return Position(x, y); }
    /** @brief 壁の方向 */
    Direction getDirection() const {
      // return z == 0 ? Direction::East : Direction::North;
      return z << 1;  //< 高速化
    }
    /** @brief 機体の方向 */
    Direction getNodeDirection() const { return nd; }
    /** @brief 直近の壁 */
    WallIndex getWallIndex() const {
      const auto nd = getNodeDirection();
      return nd.isAlong() ? WallIndex(Position(x, y), nd) : WallIndex(x, y, z);
    }
    /** @brief 表示 */
    friend std::ostream& operator<<(std::ostream& os, const Index& i);
    /**
     * @brief 斜め方向に向いているときの区画への相対方向(±45度)を返す
     * @return const Direction Direction::Left45 or Direction::Right45
     */
    Direction getRelativeDirectionDiagToAlong() const {
      switch (nd) {
        case Direction::NorthEast:
        case Direction::SouthWest:
          return z == 0 ? Direction::Left45 : Direction::Right45;
        case Direction::NorthWest:
        case Direction::SouthEast:
          return z == 1 ? Direction::Left45 : Direction::Right45;
        default:
          MAZE_LOGE << "Invalid Direction: " << nd << std::endl;
          return Direction::Max;
      }
    }
    /**
     * @brief NodeDirection が向いている方向の隣の Index を返す
     * @return const Index 隣接 Index
     */
    Index next(const Direction nd) const;
    /**
     * @brief 反対向きのIndexを取得
     * @return const Index 反対向き Index
     */
    Index opposite() const { return Index(x, y, z, nd + Direction::Back); }
  };
  static_assert(sizeof(Index) == 2, "size error");

  /**
   * @brief Index の動的配列、集合
   */
  using Indexes = std::vector<Index>;

 public:
  StepMapSlalom() {}
  /**
   * @brief ステップマップを表示する関数
   */
  void print(const Maze& maze, const Indexes& indexes,
             std::ostream& os = std::cout) const;
  /**
   * @brief 迷路上に Indexes を表示する関数
   */
  void printPath(const Maze& maze, const Indexes& indexes,
                 std::ostream& os = std::cout) const;
  /**
   * @brief コストマップの更新
   * @param maze 迷路オブジェクト
   * @param edgeCost エッジコストオブジェクト
   * @param dest 目的地の集合
   * @param knownOnly 既知壁のみを通過可能とする
   */
  void update(const Maze& maze, const EdgeCost& edgeCost, const Indexes& dest,
              const bool knownOnly);
  /**
   * @brief 最短経路を導出する
   * @param maze 迷路オブジェクト
   * @param edgeCost エッジコストオブジェクト
   * @param knownOnly 既知壁のみを通行可能とする
   */
  Directions calcShortestDirections(const Maze& maze, const EdgeCost& edgeCost,
                                    const bool knownOnly);
  /**
   * @brief コストマップを辿って経路を生成する
   * @param path 経路を格納する配列
   * @retval true 成功
   * @return false ゴールにたどりつけなかった
   */
  bool genPathFromMap(Indexes& path) const;
  /**
   * @brief コストマップから最短経路のコストを取得する
   * @return cost_t 最短経路のコスト
   */
  cost_t getShortestCost() const {
    return costMap[indexStart.opposite().getIndex()] *
           EdgeCost::RunParameter::factor;
  }

  /**
   * @brief 目的地の区画集合を Indexes に変換する関数
   * @details 目的区画のうち壁のない入射 Index を抽出。すべて区画の中央の Index
   * @param src 区画ベースの目的地配列
   * @return const Indexes
   */
  static Indexes convertDestinations(const Positions& src);
  /**
   * @brief ノード列を方向列に変換する関数
   * @param path 変換元の Index 列
   * @return const Directions
   */
  static Directions indexes2directions(const Indexes& path);

#if MAZE_DEBUG_PROFILING
  int queueSizeMax = 0;
#endif

 private:
  /** @brief スタートのノードの Index */
  const Index indexStart = Index(Position(0, 0), Direction::North);
  /** @brief 迷路上のノードのコストマップ */
  std::array<cost_t, Index::SIZE> costMap;
  /** @brief 迷路上の最短経路候補の移動元ノードを格納するマップ */
  std::array<Index, Index::SIZE> fromMap;
};

}  // namespace MazeLib
