/**
 * @file Maze.h
 * @brief マイクロマウスの迷路クラスを定義
 * @author KERI (Github: kerikun11)
 * @url https://kerikeri.top/
 * @date 2017.10.30
 */
#pragma once

#include <array>
#include <bitset>
#include <cmath> /*< for std::log2 */
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

/**
 * @brief 迷路探索ライブラリはすべてこの名前空間に格納されている．
 */
namespace MazeLib {
/**
 *  @brief 迷路の1辺の区画数の定数．2の累乗でなければならない
 */
static constexpr int MAZE_SIZE = 32;
static constexpr int MAZE_SIZE_BIT = std::log2(MAZE_SIZE);

/**
 *  @brief 迷路のカラー表示切替
 */
#if 1
#define C_RE "\x1b[31m" /*< RED */
#define C_GR "\x1b[32m" /*< GREEN */
#define C_YE "\x1b[33m" /*< YELLOW */
#define C_BL "\x1b[34m" /*< BLUE */
#define C_MA "\x1b[35m" /*< MAGENTA */
#define C_CY "\x1b[36m" /*< CYAN */
#define C_NO "\x1b[0m"  /*< RESET */
#else
#define C_RE ""
#define C_GR ""
#define C_YE ""
#define C_BL ""
#define C_MA ""
#define C_CY ""
#define C_NO ""
#endif
/** @brief カーソルの移動；マクロなので引数は定数のみ */
#define ESC_UP(n) "\x1b[" #n "A"

/**
 * @brief ログ出力用 stream
 */
#ifndef loge /**< @brief Error */
#define loge (std::cerr << "[E][" << __FILE__ << ":" << __LINE__ << "] ")
#endif
#ifndef logw /**< @brief Warning */
#define logw (std::cout << "[W][" << __FILE__ << ":" << __LINE__ << "] ")
#endif
#ifndef logi /**< @brief Info */
#define logi (std::cout << "[I][" << __FILE__ << ":" << __LINE__ << "] ")
#endif

/**
 * @brief 迷路上の方向を定義
 * 実体は 8bit の整数
 * コンストラクタによって確実に 0-7 の整数になる．
 */
struct Dir {
public:
  /**
   *  @brief 絶対方向の列挙型 0-7
   */
  enum AbsoluteDir : int8_t {
    East,
    NorthEast,
    North,
    NorthWest,
    West,
    SouthWest,
    South,
    SouthEast,
  };
  /**
   *  @brief 相対方向の列挙型 0-7
   */
  enum RelativeDir : int8_t {
    Front,
    Left45,
    Left,
    Left135,
    Back,
    Right135,
    Right,
    Right45,
  };
  /**
   * @brief 方向の総数
   */
  static constexpr int8_t Max = 8;

public:
  /**
   * @brief Construct a new Dir object
   * @param d Direction
   */
  Dir(const AbsoluteDir d = East) : d(d) {} /**< enum ならそのまま格納 */
  Dir(const int8_t d) : d(d & 7) {} /**< @brief 定義範囲内に直す */
  /** @brief 整数へのキャスト */
  operator int8_t() const { return d; }
  /** @brief 表示用char型へのキャスト */
  char toChar() const { return ">'^`<,v.X"[d]; }
  bool isAlong() const { return (d & 1) == 0; }
  bool isDiag() const { return (d & 1) == 1; }
  /**
   *  @brief 方向配列を生成する静的関数
   */
  static const std::array<Dir, 4> &ENWS() {
    static const std::array<Dir, 4> ds{East, North, West, South};
    return ds;
  }
  static const std::array<Dir, 4> &Diag4() {
    static const std::array<Dir, 4> ds{NorthEast, NorthWest, SouthWest,
                                       SouthEast};
    return ds;
  }
  /**
   * @brief ostream への表示オーバーロード
   */
  friend std::ostream &operator<<(std::ostream &os, const Dir d) {
    return os << d.toChar();
  }

private:
  int8_t d; /**< @brief 方向の実体, コンストラクタによって確実に 0-7 に収める */
};
static_assert(sizeof(Dir) == 1, "size error"); /**< size check */
/**
 *  @brief Dir構造体の動的配列
 */
using Dirs = std::vector<Dir>;

/**
 * @brief 迷路の区画の座標を定義．左下の区画が (0,0) の (x,y) 平面
 * 実体は 16bit の整数
 */
union Vector {
public:
  /**
   * @brief フィールドの区画数
   */
  static constexpr int SIZE = MAZE_SIZE * MAZE_SIZE;
  /* @brief 座標の構造体を定義 */
  struct {
    int8_t x; /**< @brief 迷路の区画座標 */
    int8_t y; /**< @brief 迷路の区画座標 */
  };
  uint16_t all; /**< @brief まとめて扱うとき用 */

public:
  /**
   * @brief Construct a new Vector object
   * @param x,y 初期化パラメータ
   */
  Vector(int8_t x, int8_t y) : x(x), y(y) {}
  Vector() : all(0) {}
  operator uint16_t() const { return (x << MAZE_SIZE_BIT) | y; }
  /**
   * @brief 演算子のオーバーロード
   */
  const Vector operator+(const Vector v) const {
    return Vector(x + v.x, y + v.y);
  }
  const Vector operator-(const Vector v) const {
    return Vector(x - v.x, y - v.y);
  }
  bool operator==(const Vector v) const { return this->all == v.all; }
  bool operator!=(const Vector v) const { return this->all != v.all; }
  /** @function next
   *  @brief 自分の引数方向に隣接した区画のVectorを返す
   *  @param 隣接方向
   *  @return 隣接座標
   */
  const Vector next(const Dir d) const;
  /**
   * @brief フィールド外かどうかを判定する関数
   * @return true フィールド外
   * @return false フィールド内
   */
  bool isOutsideofField() const {
    /* 高速化; MAZE_SIZE が2の累乗であることを使用 */
    return ((x | y) & (0x100 - MAZE_SIZE));
    // return x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE;
  }
  /**
   * @brief 座標を回転変換する
   * @param d 回転角度
   * @return const Vector
   */
  const Vector rotate(const Dir d) const;
  const Vector rotate(const Dir d, const Vector center) const {
    return center + (*this - center).rotate(d);
  }
  /**
   * @brief 表示
   */
  friend std::ostream &operator<<(std::ostream &os, const Vector v);
};
static_assert(sizeof(Vector) == 2, "size error"); /**< size check */
/**
 * @brief Vector構造体の動的配列
 */
using Vectors = std::vector<Vector>;

/**
 * @brief Vector と Dir をまとめた型
 */
using VecDir = std::pair<Vector, Dir>;
/**
 * @brief Vector と Dir を同時に表示
 */
std::ostream &operator<<(std::ostream &os, const VecDir &obj);

/**
 * @brief 区画ベースではなく，壁ベースの管理ID
 * uint16_t にキャストすると，全部の壁が通し番号になったIDを取得できるのが特徴
 */
union __attribute__((__packed__)) WallIndex {
  /**
   * @brief 壁を unique な通し番号として表現したときの総数
   */
  static constexpr int SIZE = MAZE_SIZE * MAZE_SIZE * 2;
  /**
   * @brief インデックスの構造を定義
   */
  struct {
    int8_t x : 7; /**< @brief x coordinate of the cell */
    int8_t y : 7; /**< @brief y coordinate of the cell */
    uint8_t
        z : 1; /**< @brief position assignment in the cell; 0:East,1:North */
  };

public:
  WallIndex(const int8_t x, const int8_t y, const uint8_t z)
      : x(x), y(y), z(z) {}
  WallIndex(const int8_t x, const int8_t y, const Dir d) : x(x), y(y) {
    uniquify(d);
  }
  WallIndex(const Vector v, const Dir d) : x(v.x), y(v.y) { uniquify(d); }
  WallIndex() {}
  /**
   * @brief 迷路中の壁をuniqueな通し番号として表現したID
   * @return uint16_t ID
   */
  operator uint16_t() const {
    return (z << (2 * MAZE_SIZE_BIT)) | (y << MAZE_SIZE_BIT) | x;
  }
  /**
   * @brief 方向の冗長性を除去してユニークにする関数
   *
   * @param d 壁の方向 (4方位)
   */
  void uniquify(const Dir d) {
    z = (d >> 1) & 1; /*< East,West => 0, North,South => 1 */
    if (d == Dir::West)
      x--;
    if (d == Dir::South)
      y--;
  }
  const Dir getDir() const { return z == 0 ? Dir::East : Dir::North; }
  const Vector getVector() const { return Vector(x, y); }
  /**
   * @brief 表示用演算子のオーバーロード． ( x, y, d) の形式
   */
  friend std::ostream &operator<<(std::ostream &os, const WallIndex i);
  /**
   * @brief 壁がフィールド内か判定する関数
   * x,y が (0,0)と(MAZE_SIZE-1,MAZE_SIZE-1)の間かつ，z が外周上にいない
   *
   * @return true フィールド内
   * @return false フィールド外(外周上を含む)
   */
  bool isInsideOfFiled() const {
    /* x,y が フィールド内かつ，外周上にいない */
    /* 高速化; MAZE_SIZE が2の累乗であることを使用 */
    // return (x >= 0 && y >= 0 && x < MAZE_SIZE && y < MAZE_SIZE);
    return !(((x | y) & (0x100 - MAZE_SIZE)) ||
             (z == 0 && x == MAZE_SIZE - 1) || (z == 1 && y == MAZE_SIZE - 1));
  }
  /**
   * @brief 引数方向の WallIndex を取得する関数
   *
   * @param d 隣接方向
   * @return const WallIndex 隣接壁
   */
  const WallIndex next(const Dir d) const;
  /**
   * @brief 現在壁に隣接する，柱ではない6方向を取得
   *
   * @return const std::array<Dir, 6>
   */
  const std::array<Dir, 6> getNextDir6() const {
    const auto d = getDir();
    return {d + Dir::Front,   d + Dir::Back,    d + Dir::Left45,
            d + Dir::Right45, d + Dir::Left135, d + Dir::Right135};
  }
  /**
   * @brief 引数方向の Front Left45 Right 方向に隣接する WallIndex を取得
   *
   * @param d 正面の方向
   * @return const std::array<Dir, 3>
   */
  const std::array<Dir, 3> getNextDir3(const Dir d) const {
    return {d + Dir::Front, d + Dir::Left45, d + Dir::Right45};
  }
};
static_assert(sizeof(WallIndex) == 2, "size error"); /**< size check */
/**
 * @brief WallIndex の動的配列
 *
 */
using WallIndexes = std::vector<WallIndex>;

/**
 * @brief 区画位置，方向，壁の有無を保持する構造体．実体は 16bit の整数
 */
union __attribute__((__packed__)) WallLog {
  struct __attribute__((__packed__)) {
    int x : 6;          /**< @brief 区画のx座標 */
    int y : 6;          /**< @brief 区画のx座標 */
    unsigned int d : 3; /**< @brief 壁の方向 */
    unsigned int b : 1; /**< @brief 壁の有無 */
  };
  unsigned int all : 16; /**< @brief 全フラグ参照用 */
  WallLog() {}
  WallLog(const Vector v, const Dir d, const bool b)
      : x(v.x), y(v.y), d(d), b(b) {}
  WallLog(const int8_t x, const int8_t y, const Dir d, const bool b)
      : x(x), y(y), d(d), b(b) {}
  operator Vector() const { return Vector(x, y); }
  operator Dir() const { return d; }
  friend std::ostream &operator<<(std::ostream &os, const WallLog &obj);
};
static_assert(sizeof(WallLog) == 2, "size error"); /**< size check */
/**
 * @brief WallLog構造体の動的配列
 */
using WallLogs = std::vector<WallLog>;

/**
 * @brief 迷路の壁情報を管理するクラス
 * 実体は，壁情報とスタート位置とゴール位置s
 */
class Maze {
public:
  Maze(const Vectors &goals = Vectors{}, const Vector start = Vector(0, 0))
      : goals(goals), start(start) {
    reset();
  }
  /**
   *  @brief ファイル名から迷路をパースするコンストラクタ
   *  @param filename ファイル名
   */
  Maze(const char *filename) { parse(filename); }
  /**
   *  @brief 配列から迷路を読み込むコンストラクタ
   *  @param data 各区画16進表記の文字列配列
   *  例：{"abaf", "1234", "abab", "aaff"}
   */
  Maze(const char data[MAZE_SIZE + 1][MAZE_SIZE + 1],
       const std::array<Dir, 4> bit_to_dir_map = {Dir::East, Dir::North,
                                                  Dir::West, Dir::South});
  /**
   *  @brief 迷路の初期化．壁を削除し，スタート区画を既知に
   *  @param set_start_wall スタート区画の East と North の壁を設定するかどうか
   */
  void reset(const bool set_start_wall = true);
  /**
   *  @brief 壁の有無を返す
   *  @return true: 壁あり，false: 壁なし
   */
  bool isWall(const Vector v, const Dir d) const {
    return isWallBase(wall, WallIndex(v, d));
  }
  bool isWall(const int8_t x, const int8_t y, const Dir d) const {
    return isWallBase(wall, WallIndex(x, y, d));
  }
  bool isWall(const WallIndex i) const { return isWallBase(wall, i); }
  /**
   *  @brief 壁を更新をする
   *  @param b 壁の有無 true:壁あり，false:壁なし
   */
  void setWall(const WallIndex i, const bool b) {
    return setWallBase(wall, i, b);
  }
  void setWall(const Vector v, const Dir d, const bool b) {
    return setWallBase(wall, WallIndex(v, d), b);
  }
  void setWall(const int8_t x, const int8_t y, const Dir d, const bool b) {
    return setWallBase(wall, WallIndex(x, y, d), b);
  }
  /**
   *  @brief 壁が探索済みかを返す
   *  @return true: 探索済み，false: 未探索
   */
  bool isKnown(const WallIndex i) const { return isWallBase(known, i); }
  bool isKnown(const Vector v, const Dir d) const {
    return isWallBase(known, WallIndex(v, d));
  }
  bool isKnown(const int8_t x, const int8_t y, const Dir d) const {
    return isWallBase(known, WallIndex(x, y, d));
  }
  /**
   *  @brief 壁の既知を更新する
   *  @param b 壁の未知既知 true:既知，false:未知
   */
  void setKnown(const Vector v, const Dir d, const bool b) {
    return setWallBase(known, WallIndex(v, d), b);
  }
  void setKnown(const int8_t x, const int8_t y, const Dir d, const bool b) {
    return setWallBase(known, WallIndex(x, y, d), b);
  }
  void setKnown(const WallIndex i, const bool b) {
    return setWallBase(known, i, b);
  }
  /**
   *  @brief 通過可能かどうかを返す
   *  @param v 区画の座標
   *  @param d 壁の方向
   *  @return true:既知かつ壁なし，false:それ以外
   */
  bool canGo(const Vector v, const Dir d) const {
    return isKnown(v, d) && !isWall(v, d);
  }
  bool canGo(const WallIndex i) const { return isKnown(i) && !isWall(i); }
  /**
   *  @brief 引数区画の壁の数を返す
   *  @param v 区画の座標
   *  @return 壁の数 0~4
   */
  int8_t wallCount(const Vector v) const;
  /**
   *  @brief 引数区画の未知壁の数を返す
   *  @param v 区画の座標
   *  @return 既知壁の数 0~4
   */
  int8_t unknownCount(const Vector v) const;
  /**
   *  @brief 既知の壁と照らしあわせながら，壁を更新する関数
   *         既知の壁と非一致した場合，未知壁にして return する
   *  @param v 区画の座標
   *  @param d 壁の方向
   *  @param b 壁の有無
   *  @return true: 正常に更新された, false: 既知の情報と不一致だった
   */
  bool updateWall(const Vector v, const Dir d, const bool b,
                  const bool pushLog = true);
  /**
   *  @brief 直前に更新した壁を見探索状態にリセットする
   *  @param num リセットする壁の数
   */
  void resetLastWall(const int num);
  /**
   *  @brief 迷路の表示
   *  @param of output-stream
   */
  void print(std::ostream &os = std::cout) const;
  /**
   *  @brief 特定の迷路の文字列(*.maze ファイル)から壁をパースする
   *  @param is input-stream
   */
  bool parse(std::istream &is);
  bool parse(std::string filepath) {
    std::ifstream ifs(filepath);
    if (ifs.fail())
      return false;
    return parse(ifs);
  }
  /**
   *  @brief パス付の迷路の表示
   *  @param start パスのスタート座標
   *  @param dirs 移動方向の配列
   *  @param of output-stream
   */
  void printPath(const Vector start, const Dirs &dirs,
                 std::ostream &os = std::cout) const;

  /**
   * @brief Set the Goals object
   */
  void setGoals(const Vectors &goals) { this->goals = goals; }
  /**
   * @brief Set the Start object
   */
  void setStart(const Vector &start) { this->start = start; }
  /**
   * @brief Get the Goals object
   */
  const Vectors &getGoals() const { return goals; }
  /**
   * @brief Get the Start object
   */
  const Vector &getStart() const { return start; }
  /**
   * @brief Get the Wall Logs object
   */
  const WallLogs &getWallLogs() const { return wallLogs; }
  int8_t getMinX() const { return min_x; }
  int8_t getMinY() const { return min_y; }
  int8_t getMaxX() const { return max_x; }
  int8_t getMaxY() const { return max_y; }

private:
  std::bitset<WallIndex::SIZE> wall;  /**< @brief 壁情報 */
  std::bitset<WallIndex::SIZE> known; /**< @brief 既知壁情報 */
  Vectors goals;                      /**< @brief ゴール区画 */
  Vector start;                       /**< @brief スタート区画 */
  WallLogs wallLogs;                  /**< @brief 更新した壁のログ */
  int8_t min_x;                       /**< @brief 既知壁の最小区画 */
  int8_t min_y;                       /**< @brief 既知壁の最小区画 */
  int8_t max_x;                       /**< @brief 既知壁の最大区画 */
  int8_t max_y;                       /**< @brief 既知壁の最大区画 */

  /**
   * @brief 壁の確認のベース関数
   *
   * @param wall
   * @param i
   * @return true
   * @return false
   */
  bool isWallBase(const std::bitset<WallIndex::SIZE> &wall,
                  const WallIndex i) const {
    return i.isInsideOfFiled() ? wall[i] : true;
  }
  /**
   * @brief 壁の更新のベース関数
   *
   * @param wall
   * @param i
   * @return true
   * @return false
   */
  void setWallBase(std::bitset<WallIndex::SIZE> &wall, const WallIndex i,
                   const bool b) const {
    if (i.isInsideOfFiled())
      wall[i] = b;
  }
};

} // namespace MazeLib
