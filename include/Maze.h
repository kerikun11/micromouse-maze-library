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
 * @brief 迷路の1辺の区画数の定数．2の累乗でなければならない
 */
static constexpr int MAZE_SIZE = 32;
static constexpr int MAZE_SIZE_BIT = std::log2(MAZE_SIZE);

/**
 * @brief 迷路のカラー表示切替
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
 * @brief 迷路上の方向を表す．
 * 実体は 8bit の整数．
 * 絶対方向 or 相対方向の8方位を表現することができる．
 * コンストラクタにより8方位(0-7)に自動的に収められるので，
 * 加法，減法により相対方向を計算することができる．
 * 例: Direction(Direction::East + Direction::Left) == Direction::North
 * 例: Direction(Direction::East - Direction::West) == Direction::Back
 * 例: Direction(-Direction::Left) == Direction::Right
 */
struct Direction {
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
   * @brief デフォルトコンストラクタ
   */
  Direction(const AbsoluteDirection d = East) : d(d) {}
  /**
   * @brief 整数を引数としたコンストラクタ．
   * 相対方向などの計算結果を 0-7 の整数に直して格納する．
   *
   * @param d 相対方向などの演算結果の整数
   */
  Direction(const int8_t d) : d(d & 7) {}
  /** @brief 整数へのキャスト．相対方向などの演算に使える． */
  operator int8_t() const { return d; }
  /** @brief 斜めかどうかの判定 */
  bool isAlong() const { return (d & 1) == 0; }
  bool isDiag() const { return (d & 1) == 1; }
  /** @brief 方向配列を生成する静的関数 */
  static const std::array<Direction, 4> &getAlong4() {
    static const std::array<Direction, 4> ds{{East, North, West, South}};
    return ds;
  }
  static const std::array<Direction, 4> &getDiag4() {
    static const std::array<Direction, 4> ds{
        {NorthEast, NorthWest, SouthWest, SouthEast}};
    return ds;
  }
  /** @brief 表示用char型へのキャスト */
  char toChar() const { return ">'^`<,v.X"[d]; }
  /** @brief stream での表示 */
  friend std::ostream &operator<<(std::ostream &os, const Direction d) {
    return os << d.toChar();
  }

private:
  int8_t d; /**< @brief 方向の実体, コンストラクタによって確実に 0-7 に収める */
};
static_assert(sizeof(Direction) == 1, "size error"); /**< size check */

/**
 *  @brief Direction構造体の動的配列の定義
 */
using Directions = std::vector<Direction>;

/**
 * @brief 迷路の区画の座標を定義．左下の区画が (0,0) の (x,y) 平面
 * 実体は 16bit の整数
 */
union Position {
public:
  /** @brief フィールドの区画数．配列確保などで使える． */
  static constexpr int SIZE = MAZE_SIZE * MAZE_SIZE;
  /* @brief 座標の構造体を定義 */
  struct {
    int8_t x; /**< @brief 迷路区画のx座標成分 */
    int8_t y; /**< @brief 迷路区画のy座標成分 */
  };
  uint16_t all; /**< @brief まとめて扱うとき用 */

public:
  /**
   * @brief コンストラクタ
   * @param x,y 初期化パラメータ
   */
  Position(int8_t x, int8_t y) : x(x), y(y) {}
  Position() : all(0) {}
  /**
   * @brief 迷路区画内の通し番号となるIDを取得する
   * @return uint16_t 通し番号ID
   */
  operator uint16_t() const { return (x << MAZE_SIZE_BIT) | y; }
  /** @brief 演算子のオーバーロード */
  const Position operator+(const Position p) const {
    return Position(x + p.x, y + p.y);
  }
  const Position operator-(const Position p) const {
    return Position(x - p.x, y - p.y);
  }
  bool operator==(const Position p) const { return this->all == p.all; }
  bool operator!=(const Position p) const { return this->all != p.all; }
  /**
   * @brief 自分の引数方向に隣接した区画の Position を返す
   * @param 隣接方向
   * @return 隣接座標
   */
  const Position next(const Direction d) const;
  /**
   * @brief フィールド外かどうかを判定する関数
   * @return true フィールド外
   * @return false フィールド内
   */
  bool isInsideOfField() const {
    // return x >= 0 || x < MAZE_SIZE || y >= 0 || y < MAZE_SIZE;
    /* 高速化; MAZE_SIZE が2の累乗であることを使用 */
    return !((x | y) & (0x100 - MAZE_SIZE));
  }
  /**
   * @brief 座標を回転変換する
   * @param d 回転角度, 4方位のみ
   * @return const Position
   */
  const Position rotate(const Direction d) const;
  const Position rotate(const Direction d, const Position center) const {
    return center + (*this - center).rotate(d);
  }
  /** @brief stream での表示． (  x,  y) の形式 */
  friend std::ostream &operator<<(std::ostream &os, const Position p);
};
static_assert(sizeof(Position) == 2, "size error"); /**< size check */

/**
 * @brief Position構造体の動的配列の定義
 */
using Positions = std::vector<Position>;

/**
 * @brief Position と Direction をまとめた型
 */
using Pose = std::pair<Position, Direction>;
/** @brief stream での表示 */
std::ostream &operator<<(std::ostream &os, const Pose &obj);

/**
 * @brief 区画ベースではなく，壁ベースの管理ID
 * uint16_t にキャストすると，全部の壁が通し番号になったIDを取得できるのが特徴
 * 迷路内部の壁の総数 WallIndex::SIZE 個の配列を確保しておけば，
 * 取得したIDをインデックスとして使える．そのとき， WallIndex が
 * 迷路の内部にあるかどうか確認すること．(配列の範囲外アクセス防止)
 * isInsideOfField() 関数により迷路の内部に位置するか確認できる．
 * 最初から全部が通し番号のIDで保持してしまうと，
 * 迷路の範囲外の壁を表現できなくなってしまうため，
 * 必要に応じてIDを生成するようになっている．
 */
union __attribute__((__packed__)) WallIndex {
  /**
   * @brief 壁を unique な通し番号として表現したときの総数．
   * 配列の確保などで使用できる．
   */
  static constexpr int SIZE = MAZE_SIZE * MAZE_SIZE * 2;
  /**
   * @brief インデックスの構造を定義
   */
  struct {
    int8_t x : 7;  /**< @brief 区画座標のx成分 */
    int8_t y : 7;  /**< @brief 区画座標のy成分 */
    uint8_t z : 1; /**< @brief 区画内の壁の位置．0:East,1:North */
  };

public:
  /**
   * @brief デフォルトコンストラクタ
   */
  WallIndex() {}
  WallIndex(const int8_t x, const int8_t y, const uint8_t z)
      : x(x), y(y), z(z) {}
  /**
   * @brief 表現の冗長性を除去して格納するコンストラクタ
   * @param p 区画位置
   * @param d 区画内方向．4方位
   */
  WallIndex(const Position p, const Direction d) : x(p.x), y(p.y) {
    uniquify(d);
  }
  WallIndex(const int8_t x, const int8_t y, const Direction d) : x(x), y(y) {
    uniquify(d);
  }
  /**
   * @brief 迷路中の壁をuniqueな通し番号として表現したID
   * @return uint16_t ID
   */
  operator uint16_t() const {
    return (z << (2 * MAZE_SIZE_BIT)) | (y << MAZE_SIZE_BIT) | x;
  }
  /**
   * @brief 方向の冗長性を除去してユニークにする関数
   * 基本的にコンストラクタで使われるので，ユーザーが使うことはない．
   *
   * @param d 壁の方向 (4方位)
   */
  void uniquify(const Direction d) {
    z = (d >> 1) & 1; /*< East,West => 0, North,South => 1 */
    if (d == Direction::West)
      x--;
    if (d == Direction::South)
      y--;
  }
  /** @brief Getters */
  const Direction getDirection() const {
    return z == 0 ? Direction::East : Direction::North;
  }
  const Position getPosition() const { return Position(x, y); }
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
    // return !(x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE ||
    //          (z == 0 && x == MAZE_SIZE - 1) || (z == 1 && y == MAZE_SIZE -
    //          1));
    /* 高速化; MAZE_SIZE が2の累乗であることを使用 */
    return !(((x | y) & (0x100 - MAZE_SIZE)) ||
             (z == 0 && x == MAZE_SIZE - 1) || (z == 1 && y == MAZE_SIZE - 1));
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
    return {{d + Direction::Front, d + Direction::Back, d + Direction::Left45,
             d + Direction::Right45, d + Direction::Left135,
             d + Direction::Right135}};
  }
  /**
   * @brief 引数方向の Front Left45 Right 方向に隣接する WallIndex を取得
   * @param d 正面の方向
   * @return const std::array<Direction, 3>
   */
  const std::array<Direction, 3> getNextDirection3(const Direction d) const {
    return {
        {d + Direction::Front, d + Direction::Left45, d + Direction::Right45}};
  }
};
static_assert(sizeof(WallIndex) == 2, "size error"); /**< size check */

/**
 * @brief WallIndex の動的配列の定義
 */
using WallIndexes = std::vector<WallIndex>;

/**
 * @brief 区画位置，方向，壁の有無を保持する構造体．実体は 16bit の整数
 */
union WallLog {
  struct __attribute__((__packed__)) {
    int x : 6;          /**< @brief 区画のx座標 */
    int y : 6;          /**< @brief 区画のy座標 */
    unsigned int d : 3; /**< @brief 壁の方向 */
    unsigned int b : 1; /**< @brief 壁の有無 */
  };
  /** @brief コンストラクタ */
  WallLog() {}
  WallLog(const Position p, const Direction d, const bool b)
      : x(p.x), y(p.y), d(d), b(b) {}
  WallLog(const int8_t x, const int8_t y, const Direction d, const bool b)
      : x(x), y(y), d(d), b(b) {}
  operator Position() const { return Position(x, y); }
  operator Direction() const { return d; }
  friend std::ostream &operator<<(std::ostream &os, const WallLog &obj);
};
static_assert(sizeof(WallLog) == 2, "size error"); /**< size check */

/**
 * @brief WallLog構造体の動的配列の定義
 */
using WallLogs = std::vector<WallLog>;

/**
 * @brief 迷路の壁情報を管理するクラス
 * 実体は，壁情報とスタート位置とゴール位置の集合
 * 壁の有無の確認は，isWall()
 * 壁の既知未知の確認は，isKnown()
 * 壁の更新は，updateWall()
 */
class Maze {
public:
  /**
   * @brief デフォルトコンストラクタ
   * @param goals ゴール区画の集合
   * @param start スタート区画
   */
  Maze(const Positions &goals = Positions{},
       const Position start = Position(0, 0))
      : goals(goals), start(start) {
    reset();
  }
  /**
   * @brief 迷路ファイルから迷路情報をパースするコンストラクタ
   * @param filename ファイル名
   */
  Maze(const char *filename) { parse(filename); }
  /**
   * @brief 迷路の初期化．壁を削除し，スタート区画を既知に
   * @param set_start_wall スタート区画の East と North の壁を設定するかどうか
   */
  void reset(const bool set_start_wall = true);
  /**
   * @brief 壁の有無を返す
   * @return true: 壁あり，false: 壁なし
   */
  bool isWall(const WallIndex i) const { return isWallBase(wall, i); }
  bool isWall(const Position p, const Direction d) const {
    return isWallBase(wall, WallIndex(p, d));
  }
  bool isWall(const int8_t x, const int8_t y, const Direction d) const {
    return isWallBase(wall, WallIndex(x, y, d));
  }
  /**
   * @brief 壁を更新をする
   * @param b 壁の有無 true:壁あり，false:壁なし
   */
  void setWall(const WallIndex i, const bool b) {
    return setWallBase(wall, i, b);
  }
  void setWall(const Position p, const Direction d, const bool b) {
    return setWallBase(wall, WallIndex(p, d), b);
  }
  void setWall(const int8_t x, const int8_t y, const Direction d,
               const bool b) {
    return setWallBase(wall, WallIndex(x, y, d), b);
  }
  /**
   * @brief 壁が探索済みかを返す
   * @return true: 探索済み，false: 未探索
   */
  bool isKnown(const WallIndex i) const { return isWallBase(known, i); }
  bool isKnown(const Position p, const Direction d) const {
    return isWallBase(known, WallIndex(p, d));
  }
  bool isKnown(const int8_t x, const int8_t y, const Direction d) const {
    return isWallBase(known, WallIndex(x, y, d));
  }
  /**
   * @brief 壁の既知を更新する
   * @param b 壁の未知既知 true:既知，false:未知
   */
  void setKnown(const WallIndex i, const bool b) {
    return setWallBase(known, i, b);
  }
  void setKnown(const Position p, const Direction d, const bool b) {
    return setWallBase(known, WallIndex(p, d), b);
  }
  void setKnown(const int8_t x, const int8_t y, const Direction d,
                const bool b) {
    return setWallBase(known, WallIndex(x, y, d), b);
  }
  /**
   * @brief 通過可能かどうかを返す
   * @return true:既知かつ壁なし，false:それ以外
   */
  bool canGo(const WallIndex i) const { return isKnown(i) && !isWall(i); }
  bool canGo(const Position p, const Direction d) const {
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
  bool updateWall(const Position p, const Direction d, const bool b,
                  const bool pushLog = true);
  /**
   *  @brief 直前に更新した壁を見探索状態にリセットする
   *  @param num リセットする壁の数
   */
  void resetLastWall(const int num);
  /**
   * @brief 引数区画の壁の数を返す
   * @param p 区画の座標
   * @return 壁の数 0~4
   */
  int8_t wallCount(const Position p) const;
  /**
   * @brief 引数区画に隣接する未知壁の数を返す
   * @param p 区画の座標
   * @return 既知壁の数 0~4
   */
  int8_t unknownCount(const Position p) const;
  /**
   * @brief 迷路の表示
   */
  void print(std::ostream &os = std::cout,
             const int maze_size = MAZE_SIZE) const;
  /**
   * @brief パス付の迷路の表示
   * @param start パスのスタート座標
   * @param dirs 移動方向の配列
   * @param of output-stream
   */
  void printPath(const Directions &dirs, const Position start = Position(0, 0),
                 std::ostream &os = std::cout) const;
  /**
   * @brief 特定の迷路の文字列(*.maze ファイル)から壁をパースする
   * @param is *.maze 形式のファイルの input-stream
   */
  bool parse(std::istream &is);
  bool parse(std::string filepath) {
    std::ifstream ifs(filepath);
    if (ifs.fail())
      return false;
    return parse(ifs);
  }
  /**
   * @brief 配列から迷路を読み込むパーサ
   * @param data 各区画16進表記の文字列配列
   * 例：{"abaf", "1234", "abab", "aaff"}
   */
  bool parse(const std::vector<std::string> data, const int maze_size);
  /**
   * @brief ゴール区画の集合を更新
   */
  void setGoals(const Positions &goals) { this->goals = goals; }
  /**
   * @brief スタート区画を更新
   */
  void setStart(const Position start) { this->start = start; }
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
  const WallLogs &getWallLogs() const { return wallLogs; }
  /**
   * @brief 既知部分の迷路サイズを返す．計算量を減らすために使用．
   */
  int8_t getMinX() const { return min_x; }
  int8_t getMinY() const { return min_y; }
  int8_t getMaxX() const { return max_x; }
  int8_t getMaxY() const { return max_y; }
  /**
   * @brief 壁ログをファイルに保存する関数
   */
  bool backupWallLogsFromFile(const std::string filepath) {
    std::ifstream fs(filepath, std::ifstream::ate);
    const auto size = static_cast<size_t>(fs.tellg());
    if (size / sizeof(WallLog) > getWallLogs().size()) {
      std::remove(filepath.c_str());
      backup_counter = 0;
    }
    std::ofstream of(filepath, std::ios::binary | std::ios::app);
    if (of.fail()) {
      loge << "failed to open file! " << filepath << std::endl;
      return false;
    }
    const auto &wallLog = getWallLogs();
    while (backup_counter < wallLog.size()) {
      const auto &wl = wallLog[backup_counter];
      of.write((const char *)&wl, sizeof(wl));
      backup_counter++;
    }
    return true;
  }
  /**
   * @brief 壁ログファイルから壁情報を復元する関数
   */
  bool restoreWallLogsFromFile(const std::string filepath) {
    std::ifstream f(filepath, std::ios::binary);
    if (f.fail()) {
      loge << "failed to open file! " << filepath << std::endl;
      return false;
    }
    backup_counter = 0;
    reset();
    while (!f.eof()) {
      WallLog wl;
      f.read((char *)(&wl), sizeof(WallLog));
      Position p = Position(wl.x, wl.y);
      Direction d = Direction(wl.d);
      bool b = wl.b;
      updateWall(p, d, b);
      backup_counter++;
    }
    return true;
  }
  /**
   * @brief ゴール区画内を行けるところまで直進させる方向列を追加する関数
   * @param maze 迷路の参照
   * @param shortest_dirs 追記元の方向列．これ自体に追記される．
   * @param diag_enabled 斜めありなし
   */
  static void appendStraightDirections(const Maze &maze,
                                       Directions &shortest_dirs,
                                       const bool diag_enabled);

private:
  std::bitset<WallIndex::SIZE> wall;  /**< @brief 壁情報 */
  std::bitset<WallIndex::SIZE> known; /**< @brief 壁の既知未知情報 */
  Positions goals;                    /**< @brief ゴール区画の集合 */
  Position start;                     /**< @brief スタート区画 */
  WallLogs wallLogs;                  /**< @brief 更新した壁のログ */
  int8_t min_x;                       /**< @brief 既知壁の最小区画 */
  int8_t min_y;                       /**< @brief 既知壁の最小区画 */
  int8_t max_x;                       /**< @brief 既知壁の最大区画 */
  int8_t max_y;                       /**< @brief 既知壁の最大区画 */
  size_t backup_counter; /**< 壁ログバックアップのカウンタ */

  /**
   * @brief 壁の確認のベース関数
   */
  bool isWallBase(const std::bitset<WallIndex::SIZE> &wall,
                  const WallIndex i) const {
    return i.isInsideOfFiled() ? wall[i] : true; /*< 配列範囲外アクセスの防止 */
  }
  /**
   * @brief 壁の更新のベース関数
   */
  void setWallBase(std::bitset<WallIndex::SIZE> &wall, const WallIndex i,
                   const bool b) const {
    if (i.isInsideOfFiled()) /*< 配列の範囲外アクセスの防止 */
      wall[i] = b;
  }
};

} // namespace MazeLib
