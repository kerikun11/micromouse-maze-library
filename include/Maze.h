/**
 * @file Maze.h
 * @author Ryotaro Onuki (GitHub: kerikun11)
 * @brief マイクロマウスの迷路を扱うクラスを定義
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

#include <cstring>
#include <string>

/**
 * @brief 迷路探索ライブラリはすべてこの名前空間に格納されている．
 */
namespace MazeLib {

/**
 * @brief 迷路の1辺の区画数の定数．
 */
static constexpr int MAZE_SIZE = 32;
/**
 * @brief 少数部分の切り上げ関数．
 */
constexpr int ceil(const float f) { return int(f) + (f != float(int(f))); }
/**
 * @brief 迷路の1辺の区画数の bit 数．bit shift などに用いる．
 */
static constexpr int MAZE_SIZE_BIT = ceil(std::log2(MAZE_SIZE));
/**
 * @brief 迷路の1辺の区画数の最大値．2のbit数乗の値．
 */
static constexpr int MAZE_SIZE_MAX = std::pow(2, MAZE_SIZE_BIT);

/*
 * 迷路のカラー表示切替
 */
#ifdef MAZE_COLOR_DISABLED
#define C_RE ""
#define C_GR ""
#define C_YE ""
#define C_BL ""
#define C_MA ""
#define C_CY ""
#define C_NO ""
#else
#define C_RE "\x1b[31m" /**< @brief ANSI Escape Sequence RED */
#define C_GR "\x1b[32m" /**< @brief ANSI Escape Sequence GREEN */
#define C_YE "\x1b[33m" /**< @brief ANSI Escape Sequence YELLOW */
#define C_BL "\x1b[34m" /**< @brief ANSI Escape Sequence BLUE */
#define C_MA "\x1b[35m" /**< @brief ANSI Escape Sequence MAGENTA */
#define C_CY "\x1b[36m" /**< @brief ANSI Escape Sequence CYAN */
#define C_NO "\x1b[0m"  /**< @brief ANSI Escape Sequence RESET */
#endif

/*
 * ログ出力 stream
 */
/** @brief Log Stream (Error) */
#ifndef loge
#if 1
#define loge (std::cout << C_RE "[E][" __FILE__ ":" << __LINE__ << "]\t" C_NO)
#else
#define loge std::ostream(0)
#endif
#endif
/** @brief Log Stream (Warnning) */
#ifndef logw
#if 1
#define logw (std::cout << C_YE "[W][" __FILE__ ":" << __LINE__ << "]\t" C_NO)
#else
#define logw std::ostream(0)
#endif
#endif
/** @brief Log Stream (Info) */
#ifndef logi
#if 1
#define logi (std::cout << C_GR "[I][" __FILE__ ":" << __LINE__ << "]\t" C_NO)
#else
#define logi std::ostream(0)
#endif
#endif

/**
 * @brief 迷路上の方向を表す．
 *
 * 実体は 8bit の整数．
 * 絶対方向 or 相対方向の8方位を表現することができる．
 * コンストラクタにより8方位(0-7)に自動的に収められるので，
 * 加法，減法により相対方向を計算することができる．
 * - 例: Direction(Direction::East + Direction::Left) == Direction::North
 * - 例: Direction(Direction::East - Direction::West) == Direction::Back
 * - 例: Direction(-Direction::Left) == Direction::Right
 * ```
 * AbsoluteDirection
 * +-----------+-------+-----------+
 * | NorthWest   North   NorthEast |
 * +           +       +           +
 * |      West     X          East |
 * +           +       +           +
 * | NorthWest   North   NorthEast |
 * +-----------+-------+-----------+
 * RelativeDirection
 * +-----------+-------+-----------+
 * |   Left135    Left      Left45 |
 * +           +       +           +
 * |      Back     X         Front |
 * +           +       +           +
 * |  Right135   Right     Right45 |
 * +-----------+-------+-----------+
 * ```
 */
class Direction {
public:
  /**
   * @brief 絶対方向の列挙型． 0-7 の整数
   */
  enum AbsoluteDirection : int8_t {
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
   * @brief 相対方向の列挙型． 0-7 の整数
   */
  enum RelativeDirection : int8_t {
    Front,
    Left45,
    Left,
    Left135,
    Back,
    Right135,
    Right,
    Right45,
  };
  /** @brief 方向の総数．for文などで使える． */
  static constexpr int8_t Max = 8;

public:
  /**
   * @brief デフォルトコンストラクタ．そのまま格納．
   */
  Direction(const AbsoluteDirection d = East) : d(d) {}
  /**
   * @brief 整数を引数としたコンストラクタ．
   * 相対方向などの計算結果を 0-7 の整数に直して格納する．
   * @param d 相対方向などの演算結果の整数
   */
  Direction(const int8_t d) : d(d & 7) {}
  /**
   * @brief 整数へのキャスト．相対方向などの演算に使える．
   */
  operator const int8_t &() const { return d; }
  /** @brief 斜めかどうかの判定 */
  bool isAlong() const { return (d & 1) == 0; }
  bool isDiag() const { return (d & 1) == 1; }
  /** @brief 表示用char型へのキャスト */
  char toChar() const { return ">'^`<,v.X"[d]; }
  /** @brief stream での表示 */
  friend std::ostream &operator<<(std::ostream &os, const Direction d) {
    return os << d.toChar();
  }
  /**
   * @brief 斜めでない絶対4方向を取得
   */
  static const std::array<Direction, 4> &getAlong4() {
    static const std::array<Direction, 4> ds{{East, North, West, South}};
    return ds;
  }
  /**
   * @brief 斜めの絶対4方向を取得
   */
  static const std::array<Direction, 4> &getDiag4() {
    static const std::array<Direction, 4> ds{
        {NorthEast, NorthWest, SouthWest, SouthEast}};
    return ds;
  }

private:
  int8_t d; /**< @brief 方向の実体, コンストラクタによって確実に 0-7 に収める */
};
static_assert(sizeof(Direction) == 1, "size error"); /**< @brief size check */

/**
 *  @brief Direction 構造体の動的配列，集合
 */
using Directions = std::vector<Direction>;

/**
 * @brief 迷路の区画の位置(座標)を定義．
 *
 * 左下の区画が (0,0) の (x,y) 平面．
 * 実体は 16bit の整数．
 * ```
 * +--------+--------+
 * | (0, 1) | (1, 1) |
 * +--------+--------+
 * | (0, 0) | (1, 0) |
 * +--------+--------+
 * ```
 */
struct Position {
public:
  /** @brief フィールドの区画数．配列確保などで使える． */
  static constexpr int SIZE = MAZE_SIZE_MAX * MAZE_SIZE_MAX;

public:
  int8_t x; /**< @brief 迷路区画のx座標成分 */
  int8_t y; /**< @brief 迷路区画のy座標成分 */

public:
  /**
   * @brief デフォルトコンストラクタ
   * @param x,y 初期化パラメータ
   */
  Position(const int8_t x = 0, const int8_t y = 0) : x(x), y(y) {}
  /**
   * @brief 迷路内の区画の一意な通し番号となるIDを取得する
   *
   * 迷路外の区画の場合未定義動作となる．use Position::isInsideOfField()
   * @return uint16_t 通し番号ID
   */
  uint16_t getIndex() const { return (x << MAZE_SIZE_BIT) | y; }
  /** @brief 加法 */
  const Position operator+(const Position &p) const {
    return Position(x + p.x, y + p.y);
  }
  /** @brief 減法 */
  const Position operator-(const Position &p) const {
    return Position(x - p.x, y - p.y);
  }
  /** @brief 等号 */
  bool operator==(const Position &p) const { return x == p.x && y == p.y; }
  /** @brief 等号否定 */
  bool operator!=(const Position &p) const { return x != p.x || y != p.y; }
  /**
   * @brief 自分の引数方向に隣接した区画の Position を返す
   * @param d 隣接方向
   * @return 隣接区画の座標
   */
  const Position next(const Direction d) const;
  /**
   * @brief フィールド内かどうかを判定する関数
   * @return true フィールド内
   * @return false フィールド外
   */
  bool isInsideOfField() const {
    // return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
    /* 高速化 */
    return (static_cast<uint8_t>(x) < MAZE_SIZE) &&
           (static_cast<uint8_t>(y) < MAZE_SIZE);
  }
  /**
   * @brief 座標を回転変換する
   * @param d 回転角度, 4方位のみ
   * @return const Position
   */
  const Position rotate(const Direction d) const;
  const Position rotate(const Direction d, const Position &center) const {
    return center + (*this - center).rotate(d);
  }
  /**
   * @brief stream での表示． (  x,  y) の形式
   */
  friend std::ostream &operator<<(std::ostream &os, const Position &p);
};
static_assert(sizeof(Position) == 2, "size error"); /**< @brief size check */

/**
 * @brief Position 構造体の動的配列，集合
 */
using Positions = std::vector<Position>;

/**
 * @brief Position と Direction をまとめた型．位置姿勢．
 *
 * 位置姿勢は，区画とそこに向かう方向で特定する．
 * 現在区画から出る方向ではないことに注意する．
 * ```
 * +---+---+---+ 例:
 * |   <       | <--- (0, 2, West)
 * +   +---+ ^ + <--- (2, 2, North)
 * |   >       | <--- (1, 1, East)
 * +   +---+ v + <--- (2, 0, South)
 * | S |       | <--- (0, 0)
 * +---+---+---+
 * ```
 */
struct Pose {
public:
  Position p;  /**< @brief 位置 */
  Direction d; /**< @brief 姿勢 */

public:
  Pose() {}
  Pose(const Position &p, const Direction d) : p(p), d(d) {}
  /**
   * @brief 隣接姿勢の取得
   * @param next_direction 隣接方向
   * @return const Pose 隣接姿勢
   */
  const Pose next(const Direction next_direction) const {
    return Pose(p.next(next_direction), next_direction);
  }
  /** @brief stream での表示 */
  friend std::ostream &operator<<(std::ostream &os, const Pose &pose);
};

/**
 * @brief 区画ベースではなく，壁ベースの管理ID
 *
 * uint16_t にキャストすると，全部の壁が通し番号になったIDを取得できるのが特徴
 * 迷路内部の壁の総数 WallIndex::SIZE 個の配列を確保しておけば，
 * 取得したIDをインデックスとして使える．そのとき， WallIndex が
 * 迷路の内部にあるかどうか確認すること．(配列の範囲外アクセス防止)
 * isInsideOfField() 関数により迷路の内部に位置するか確認できる．
 * 最初から全部が通し番号のIDで保持してしまうと，
 * 迷路の範囲外の壁を表現できなくなってしまうため，
 * 必要に応じてIDを生成するようになっている．
 * ```
 *      [x, y]    : Cell Position
 *             z  : Wall Distinction in the Cell; 0:East, 1:North
 *   => (x, y, z) : Wall Index
 * +-------------+-------------+-------------+
 * |             |             |             |
 * |             |             |             |
 * |             |             |             |
 * +-------------+- (x, y, 1) -+-------------+
 * |             |             |             |
 * |    (x-1, y, 0)  [ x, y]  (x, y, 0)      |
 * |             |             |             |
 * +--- z = 1 ---+- (x,y-1,1) -+-------------+
 * |             |             |             |
 * |    Cell   z = 0           |             |
 * |             |             |             |
 * +-------------+-------------+-------------+
 * ```
 */
struct WallIndex {
  /**
   * @brief 壁を unique な通し番号として表現したときの総数．
   * 配列の確保などで使用できる．
   */
  static constexpr int SIZE = MAZE_SIZE_MAX * MAZE_SIZE_MAX * 2;

public:
  int8_t x;      /**< @brief 区画座標のx成分 */
  int8_t y : 7;  /**< @brief 区画座標のy成分 */
  uint8_t z : 1; /**< @brief 区画内の壁の位置．0:East, 1:North */

public:
  /**
   * @brief 成分を受け取ってそのまま格納するコンストラクタ
   */
  WallIndex(const int8_t x = 0, const int8_t y = 0, const uint8_t z = 0)
      : x(x), y(y), z(z) {}
  /**
   * @brief 表現の冗長性を除去して格納するコンストラクタ
   * @param p 区画位置
   * @param d 区画内方向．4方位
   */
  WallIndex(const Position &p, const Direction d) : x(p.x), y(p.y) {
    uniquify(d);
  }
  /**
   * @brief IDを使って初期化するコンストラクタ
   * @param i 壁の通し番号ID．迷路内の壁であること．
   *          迷路外の壁の場合未定義動作となる．
   */
  WallIndex(const uint16_t i)
      : x(i & (MAZE_SIZE_MAX - 1)),
        y((i >> MAZE_SIZE_BIT) & (MAZE_SIZE_MAX - 1)),
        z(i >> (2 * MAZE_SIZE_BIT)) {}
  /** @brief 等号 */
  bool operator==(const WallIndex &i) const {
    return x == i.x && y == i.y && z == i.z;
  }
  /** @brief 等号否定 */
  bool operator!=(const WallIndex &i) const {
    return x != i.x || y != i.y || z != i.z;
  }
  /**
   * @brief 迷路内の壁を一意な通し番号として表現したIDを返す．
   *        迷路外の壁の場合未定義動作となる．
   * @return uint16_t ID
   */
  uint16_t getIndex() const {
    return (z << (2 * MAZE_SIZE_BIT)) | (y << MAZE_SIZE_BIT) | x;
  }
  /** @brief 位置の取得 */
  const Position getPosition() const { return Position(x, y); }
  /** @brief 方向の取得 */
  const Direction getDirection() const {
    // return z == 0 ? Direction::East : Direction::North;
    return z << 1; /*< 高速化 */
  }
  /**
   * @brief 表示用演算子のオーバーロード． ( x, y, d) の形式
   */
  friend std::ostream &operator<<(std::ostream &os, const WallIndex &i);
  /**
   * @brief 壁がフィールド内か判定する関数
   * x,y が (0,0)と(MAZE_SIZE-1,MAZE_SIZE-1)の間かつ，z が外周上にいない
   *
   * @return true フィールド内
   * @return false フィールド外(外周上を含む)
   */
  bool isInsideOfField() const {
    /* x,y が フィールド内かつ，外周上にいない */
    // return !(x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE ||
    //          (z == 0 && (x == MAZE_SIZE - 1)) ||
    //          (z == 1 && (y == MAZE_SIZE - 1)));
    /* 高速化 */
    return (static_cast<uint8_t>(x) < MAZE_SIZE - 1 + z) &&
           (static_cast<uint8_t>(y) < MAZE_SIZE - z);
  }
  /**
   * @brief 引数方向の WallIndex を取得する関数
   * @param d 隣接方向
   * @return const WallIndex 隣接壁
   */
  const WallIndex next(const Direction d) const;
  /**
   * @brief 現在壁に隣接する，柱ではない6方向を取得
   * @return const std::array<Direction, 6>
   */
  const std::array<Direction, 6> getNextDirection6() const {
    const auto d = getDirection();
    return {{
        d + Direction::Front,
        d + Direction::Back,
        d + Direction::Left45,
        d + Direction::Right45,
        d + Direction::Left135,
        d + Direction::Right135,
    }};
  }

private:
  /**
   * @brief 方向の冗長性を除去してユニークにする関数
   * 基本的にコンストラクタで使われるので，ユーザーが使うことはない．
   * @param d 壁の方向 (4方位)
   */
  void uniquify(const Direction d) {
    z = (d >> 1) & 1; /*< East,West => 0, North,South => 1 */
    switch (d) {
    case Direction::West:
      x--;
      break;
    case Direction::South:
      y--;
      break;
    default:
      break;
    }
  }
};
static_assert(sizeof(WallIndex) == 2, "size error"); /**< @brief size check */

/**
 * @brief WallIndex の動的配列，集合
 */
using WallIndexes = std::vector<WallIndex>;

/**
 * @brief 区画位置，方向，壁の有無を保持する構造体．
 *
 * - 探索の記録などに用いる
 * - サイズを小さくするためにビットフィールド構造体を用いている
 * - 実体は 16bit の整数
 */
struct WallRecord {
  int x : 6;          /**< @brief 区画のx座標 */
  int y : 6;          /**< @brief 区画のy座標 */
  unsigned int d : 3; /**< @brief 壁の方向 */
  unsigned int b : 1; /**< @brief 壁の有無 */
  /**
   * @brief コンストラクタ
   */
  WallRecord() {}
  WallRecord(const int8_t x, const int8_t y, const Direction d, const bool b)
      : x(x), y(y), d(d), b(b) {}
  WallRecord(const Position &p, const Direction d, const bool b)
      : x(p.x), y(p.y), d(d), b(b) {}
  /** @brief 区画の取得 */
  const Position getPosition() const { return Position(x, y); }
  /** @brief 方向の取得 */
  const Direction getDirection() const { return d; }
  /** @brief 表示 */
  friend std::ostream &operator<<(std::ostream &os, const WallRecord &obj);
} __attribute__((__packed__));
static_assert(sizeof(WallRecord) == 2, "size error"); /**< @brief size check */

/**
 * @brief WallRecord 構造体の動的配列の定義
 */
using WallRecords = std::vector<WallRecord>;

/**
 * @brief 迷路の壁情報を管理するクラス
 *
 * - 壁情報とスタート位置とゴール位置の集合などを保持する
 * - 壁の有無の確認は，isWall()
 * - 壁の既知未知の確認は，isKnown()
 * - 壁の更新は，updateWall() によって行う
 */
class Maze {
public:
  /**
   * @brief デフォルトコンストラクタ
   * @param goals ゴール区画の集合
   * @param start スタート区画
   */
  Maze(const Positions &goals = Positions(),
       const Position &start = Position(0, 0))
      : goals(goals), start(start) {
    reset();
  }
  /**
   * @brief 迷路の初期化．壁を削除し，スタート区画を既知に
   * @param set_start_wall スタート区画の East と North の壁を設定するかどうか
   * @param set_range_full 高速化用の min_x, max_x を予め最大に設定するかどうか
   */
  void reset(const bool set_start_wall = true,
             const bool set_range_full = false);
  /**
   * @brief 壁の有無を返す
   * @return true: 壁あり，false: 壁なし
   */
  bool isWall(const WallIndex &i) const { return isWallBase(wall, i); }
  bool isWall(const Position &p, const Direction d) const {
    return isWallBase(wall, WallIndex(p, d));
  }
  bool isWall(const int8_t x, const int8_t y, const Direction d) const {
    return isWallBase(wall, WallIndex(Position(x, y), d));
  }
  /**
   * @brief 壁を更新をする
   * @param b 壁の有無 true:壁あり，false:壁なし
   */
  void setWall(const WallIndex &i, const bool b) {
    return setWallBase(wall, i, b);
  }
  void setWall(const Position &p, const Direction d, const bool b) {
    return setWallBase(wall, WallIndex(p, d), b);
  }
  void setWall(const int8_t x, const int8_t y, const Direction d,
               const bool b) {
    return setWallBase(wall, WallIndex(Position(x, y), d), b);
  }
  /**
   * @brief 壁が探索済みかを返す
   * @return true: 探索済み，false: 未探索
   */
  bool isKnown(const WallIndex &i) const { return isWallBase(known, i); }
  bool isKnown(const Position &p, const Direction d) const {
    return isWallBase(known, WallIndex(p, d));
  }
  bool isKnown(const int8_t x, const int8_t y, const Direction d) const {
    return isWallBase(known, WallIndex(Position(x, y), d));
  }
  /**
   * @brief 壁の既知を更新する
   * @param b 壁の未知既知 true:既知，false:未知
   */
  void setKnown(const WallIndex &i, const bool b) {
    return setWallBase(known, i, b);
  }
  void setKnown(const Position &p, const Direction d, const bool b) {
    return setWallBase(known, WallIndex(p, d), b);
  }
  void setKnown(const int8_t x, const int8_t y, const Direction d,
                const bool b) {
    return setWallBase(known, WallIndex(Position(x, y), d), b);
  }
  /**
   * @brief 通過可能かどうかを返す
   * @return true:既知かつ壁なし，false:それ以外
   */
  bool canGo(const WallIndex &i) const { return isKnown(i) && !isWall(i); }
  bool canGo(const Position &p, const Direction d) const {
    return canGo(WallIndex(p, d));
  }
  /**
   * @brief 既知の壁情報と照らしあわせながら，壁を更新する関数
   *        既知の壁と非一致した場合，未知壁にして return する
   * @param p 区画の座標
   * @param d 壁の方向
   * @param b 壁の有無
   * @return true: 正常に更新された, false: 既知の情報と不一致だった
   */
  bool updateWall(const Position &p, const Direction d, const bool b,
                  const bool pushLog = true);
  /**
   *  @brief 直前に更新した壁を見探索状態にリセットする
   *  @param num リセットする壁の数
   */
  void resetLastWalls(const int num);
  /**
   * @brief 引数区画の壁の数を返す
   * @param p 区画の座標
   * @return 壁の数 0~4
   */
  int8_t wallCount(const Position &p) const;
  /**
   * @brief 引数区画に隣接する未知壁の数を返す
   * @param p 区画の座標
   * @return 既知壁の数 0~4
   */
  int8_t unknownCount(const Position &p) const;
  /**
   * @brief 迷路の表示
   */
  void print(std::ostream &os = std::cout,
             const int maze_size = MAZE_SIZE) const;
  /**
   * @brief パス付きの迷路の表示
   * @param start パスのスタート座標
   * @param dirs 移動方向の配列
   * @param of output-stream
   */
  void print(const Directions &dirs, const Position &start = Position(0, 0),
             std::ostream &os = std::cout,
             const size_t maze_size = MAZE_SIZE) const;
  /**
   * @brief 位置のハイライト付きの迷路の表示
   * @param positions ハイライト位置s
   */
  void print(const Positions &positions, std::ostream &os = std::cout,
             const size_t maze_size = MAZE_SIZE) const;
  /**
   * @brief 特定の迷路の文字列(*.maze ファイル)から壁をパースする
   *
   * テキスト形式．S: スタート区画(単数)，G: ゴール区画(複数)
   * ```
   * +---+---+
   * |     G |
   * +   +   +
   * | S | G |
   * +---+---+
   * ```
   * @param is *.maze 形式のファイルの input-stream
   */
  bool parse(std::istream &is);
  bool parse(std::string filepath) {
    std::ifstream ifs(filepath);
    return ifs ? parse(ifs) : false;
  }
  /**
   * @brief 配列から迷路を読み込むパーサ
   * @param data 各区画16進表記の文字列配列
   * 例：{"abaf", "1234", "abab", "aaff"}
   */
  bool parse(const std::vector<std::string> &data, const int maze_size);
  /**
   * @brief ゴール区画の集合を更新
   */
  void setGoals(const Positions &goals) { this->goals = goals; }
  /**
   * @brief スタート区画を更新
   */
  void setStart(const Position &start) { this->start = start; }
  /**
   * @brief ゴール区画の集合を取得
   */
  const Positions &getGoals() const { return goals; }
  /**
   * @brief スタート区画を取得
   */
  const Position &getStart() const { return start; }
  /**
   * @brief 壁ログを取得
   */
  const WallRecords &getWallRecords() const { return wallRecords; }
  /**
   * @brief 既知部分の迷路サイズを返す．計算量を減らすために使用．
   */
  const int8_t &getMinX() const { return min_x; }
  const int8_t &getMinY() const { return min_y; }
  const int8_t &getMaxX() const { return max_x; }
  const int8_t &getMaxY() const { return max_y; }
  /**
   * @brief 壁ログをファイルに追記保存する関数
   */
  bool backupWallRecordsToFile(const std::string &filepath,
                               const bool clear = false);
  /**
   * @brief 壁ログファイルから壁情報を復元する関数
   */
  bool restoreWallRecordsFromFile(const std::string &filepath);

protected:
  std::bitset<WallIndex::SIZE> wall;  /**< @brief 壁情報 */
  std::bitset<WallIndex::SIZE> known; /**< @brief 壁の既知未知情報 */
  Positions goals;                    /**< @brief ゴール区画の集合 */
  Position start;                     /**< @brief スタート区画 */
  WallRecords wallRecords;            /**< @brief 更新した壁のログ */
  int8_t min_x;                       /**< @brief 既知壁の最小区画 */
  int8_t min_y;                       /**< @brief 既知壁の最小区画 */
  int8_t max_x;                       /**< @brief 既知壁の最大区画 */
  int8_t max_y;                       /**< @brief 既知壁の最大区画 */
  size_t backup_counter; /**< @brief 壁ログバックアップのカウンタ */

  /**
   * @brief 壁の確認のベース関数．迷路外を参照すると壁ありと返す．
   */
  bool isWallBase(const std::bitset<WallIndex::SIZE> &wall,
                  const WallIndex &i) const {
    return i.isInsideOfField() ? wall[i.getIndex()] : true; //< 範囲外は壁ありに
  }
  /**
   * @brief 壁の更新のベース関数．迷路外を参照しても無視される．
   */
  void setWallBase(std::bitset<WallIndex::SIZE> &wall, const WallIndex &i,
                   const bool b) const {
    if (i.isInsideOfField()) //< 範囲外アクセスの防止
      wall[i.getIndex()] = b;
  }
};

} // namespace MazeLib
