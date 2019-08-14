/**
 * @file Maze.cpp
 * @brief マイクロマウスの迷路クラスを定義
 * @author KERI (Github: kerikun11)
 * @url https://kerikeri.top/
 * @date 2017.10.30
 */
#include "Maze.h"

#include <algorithm>
#include <cstdio>
#include <iomanip> //< for std::setw()

namespace MazeLib {

/* Vector */
const Vector Vector::rotate(const Dir d) const {
  switch (d) {
  case Dir::East:
    return Vector(x, y);
  case Dir::North:
    return Vector(-y, x);
  case Dir::West:
    return Vector(-x, -y);
  case Dir::South:
    return Vector(y, -x);
  }
  loge << "invalid direction" << std::endl;
  return *this;
}
std::ostream &operator<<(std::ostream &os, const Vector v) {
  return os << "(" << std::setw(2) << (int)v.x << ", " << std::setw(2)
            << (int)v.y << ")";
}

/* VecDir */
std::ostream &operator<<(std::ostream &os, const VecDir &obj) {
  return os << "( " << std::setw(2) << (int)obj.first.x << ", " << std::setw(2)
            << (int)obj.first.y << ", " << obj.second.toChar() << ")";
}

/* WallLog */
std::ostream &operator<<(std::ostream &os, const WallLog &obj) {
  return os << "( " << std::setw(2) << (int)obj.x << ", " << std::setw(2)
            << (int)obj.y << ", " << Dir(obj).toChar() << ", "
            << (obj.b ? "true" : "false") << ")";
}

/* Maze */
Maze::Maze(const char data[MAZE_SIZE + 1][MAZE_SIZE + 1],
           const std::array<Dir, 4> bit_to_dir_map) {
  for (int8_t y = 0; y < MAZE_SIZE; ++y)
    for (int8_t x = 0; x < MAZE_SIZE; ++x) {
      const char c = data[MAZE_SIZE - y - 1][x];
      // const char c = data[x][y];
      uint8_t h = 0;
      if ('0' <= c && c <= '9')
        h = c - '0';
      else if ('a' <= c && c <= 'f')
        h = c - 'a' + 10;
      else if ('A' <= c && c <= 'F')
        h = c - 'A' + 10;
      else if (0 <= c && c <= 15)
        h = c;
      updateWall(Vector(x, y), bit_to_dir_map[0], h & 0x01, false);
      updateWall(Vector(x, y), bit_to_dir_map[1], h & 0x02, false);
      updateWall(Vector(x, y), bit_to_dir_map[2], h & 0x04, false);
      updateWall(Vector(x, y), bit_to_dir_map[3], h & 0x08, false);
    }
}
void Maze::reset(const bool setStartWall) {
  for (int8_t i = 0; i < MAZE_SIZE - 1; ++i) {
    wall[0][i] = 0;
    wall[1][i] = 0;
    known[0][i] = 0;
    known[1][i] = 0;
  }
  min_x = MAZE_SIZE - 1;
  min_y = MAZE_SIZE - 1;
  max_x = 0;
  max_y = 0;
  if (setStartWall) {
    updateWall(Vector(0, 0), Dir::East, true);   //< start cell
    updateWall(Vector(0, 0), Dir::North, false); //< start cell
  }
  wallLogs.clear();
}
int8_t Maze::wallCount(const Vector v) const {
  auto dirs = Dir::ENWS();
  return std::count_if(dirs.cbegin(), dirs.cend(),
                       [&](const Dir d) { return isWall(v, d); });
}
int8_t Maze::unknownCount(const Vector v) const {
  const auto dirs = Dir::ENWS();
  return std::count_if(dirs.cbegin(), dirs.cend(),
                       [&](const Dir d) { return !isKnown(v, d); });
}
bool Maze::updateWall(const Vector v, const Dir d, const bool b,
                      const bool pushLog) {
  /* 既知の壁と食い違いがあったら未知壁としてreturn */
  if (isKnown(v, d) && isWall(v, d) != b) {
    setWall(v, d, false);
    setKnown(v, d, false);
    /* ログに追加 */
    if (pushLog)
      wallLogs.push_back(WallLog(v, d, b));
    /* ログから消去 */
    // const auto it =
    //     std::find_if(wallLogs.cbegin(), wallLogs.cend(), [&](const auto w)
    //     {
    //       return Vector(w) == v && Dir(w) == d;
    //     });
    // if (it != wallLogs.end())
    //   wallLogs.erase(it);
    return false;
  }
  /* 未知壁なら壁情報を更新 */
  if (!isKnown(v, d)) {
    setWall(v, d, b);
    setKnown(v, d, true);
    /* ログに追加 */
    if (pushLog)
      wallLogs.push_back(WallLog(v, d, b));
    /* 最大最小区画を更新 */
    min_x = std::min(v.x, min_x);
    min_y = std::min(v.y, min_y);
    max_x = std::max(v.x, max_x);
    max_y = std::max(v.y, max_y);
  }
  return true;
}
void Maze::resetLastWall(const int num) {
  for (int i = 0; i < num; ++i) {
    if (wallLogs.empty())
      return;
    const auto wl = wallLogs.back();
    setWall(Vector(wl), wl.d, false);
    setKnown(Vector(wl), wl.d, false);
    wallLogs.pop_back();
  }
  return;
}
void Maze::print(std::ostream &os) const {
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (int8_t x = 0; x < MAZE_SIZE; ++x) {
        const auto v = Vector(x, y);
        if (v == start)
          os << " S ";
        else if (std::find(goals.cbegin(), goals.cend(), v) != goals.end())
          os << " G ";
        else
          os << "   ";
        os << (isKnown(x, y, Dir::East) ? (isWall(x, y, Dir::East) ? "|" : " ")
                                        : ".");
      }
      os << std::endl;
    }
    for (int8_t x = 0; x < MAZE_SIZE; ++x)
      os << "+"
         << (isKnown(x, y, Dir::South)
                 ? (isWall(x, y, Dir::South) ? "---" : "   ")
                 : " . ");
    os << "+" << std::endl;
  }
}
bool Maze::parse(std::istream &is) {
  is.seekg(0, std::ios_base::end);
  int file_size = is.tellg();
  is.seekg(0, std::ios_base::beg);
  int maze_size = 0;
  if (file_size > 8000)
    maze_size = 32;
  else if (file_size > 2000)
    maze_size = 16;
  else
    maze_size = 8;
  reset();
  goals.clear();
  for (int8_t y = maze_size; y >= 0; --y) {
    if (y != maze_size) {
      is.ignore(10, '|'); //< 次の|が出てくるまでスキップ
      for (int8_t x = 0; x < maze_size; ++x) {
        is.ignore(1); //< " " 空欄分をスキップ
        char c = is.get();
        if (c == 'S')
          start = Vector(x, y);
        else if (c == 'G')
          goals.push_back(Vector(x, y));
        is.ignore(1); //< " " 空欄分をスキップ
        c = is.get();
        if (c == '|')
          Maze::updateWall(Vector(x, y), Dir::East, true);
        else if (c == ' ')
          Maze::updateWall(Vector(x, y), Dir::East, false);
      }
    }
    for (uint8_t x = 0; x < maze_size; ++x) {
      is.ignore(10, '+'); //< 次の+が出てくるまでスキップ
      std::string s;
      s += (char)is.get();
      s += (char)is.get();
      s += (char)is.get();
      if (s == "---")
        Maze::updateWall(Vector(x, y), Dir::South, true);
      else if (s == "   ")
        Maze::updateWall(Vector(x, y), Dir::South, false);
    }
  }
  return true;
}
void Maze::printPath(std::ostream &os, const Vector start,
                     const Dirs &dirs) const {
  int steps[MAZE_SIZE][MAZE_SIZE] = {0};
  Vector v = start;
  int counter = 1;
  for (const auto d : dirs) {
    v = v.next(d);
    if (v.isOutsideofField()) {
      loge << "Out of Field! " << v << std::endl;
      continue;
    }
    steps[v.y][v.x] = counter++;
  }
  for (int8_t y = MAZE_SIZE; y >= 0; --y) {
    if (y != MAZE_SIZE) {
      os << '|';
      for (int8_t x = 0; x < MAZE_SIZE; ++x) {
        if (steps[y][x] != 0)
          os << C_YELLOW << std::setw(3) << steps[y][x] << C_RESET;
        else
          os << "   ";
        os << (isKnown(x, y, Dir::East) ? (isWall(x, y, Dir::East) ? "|" : " ")
                                        : (C_RED "." C_RESET));
      }
      os << std::endl;
    }
    for (int8_t x = 0; x < MAZE_SIZE; ++x)
      os << "+"
         << (isKnown(x, y, Dir::South)
                 ? (isWall(x, y, Dir::South) ? "---" : "   ")
                 : (C_RED " . " C_RESET));
    os << "+" << std::endl;
  }
}
bool Maze::isWall(const wall_size_t wall[2][MAZE_SIZE - 1], const int8_t x,
                  const int8_t y, const Dir d) {
  switch (d) {
  case Dir::East:
    if (x < 0 || x >= MAZE_SIZE - 1 || y < 0 || y >= MAZE_SIZE)
      return true; //< 盤面外
    return wall[0][x] & (1 << y);
  case Dir::North:
    if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE - 1)
      return true; //< 盤面外
    return wall[1][y] & (1 << x);
  case Dir::West:
    if (x < 1 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE)
      return true; //< 盤面外
    return wall[0][x - 1] & (1 << y);
  case Dir::South:
    if (x < 0 || x >= MAZE_SIZE || y < 1 || y >= MAZE_SIZE)
      return true; //< 盤面外
    return wall[1][y - 1] & (1 << x);
  }
  loge << "invalid direction" << std::endl;
  return true; //< とりあえず壁ありとする
}
void Maze::setWall(wall_size_t wall[2][MAZE_SIZE - 1], const int8_t x,
                   const int8_t y, const Dir d, const bool b) {
  switch (d) {
  case Dir::East:
    if (x < 0 || x >= MAZE_SIZE - 1 || y < 0 || y >= MAZE_SIZE)
      return; //< 盤面外
    if (b)
      wall[0][x] |= (1 << y);
    else
      wall[0][x] &= ~(1 << y);
    return;
  case Dir::North:
    if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE - 1)
      return; //< 盤面外
    if (b)
      wall[1][y] |= (1 << x);
    else
      wall[1][y] &= ~(1 << x);
    return;
  case Dir::West:
    if (x < 1 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE)
      return; //< 盤面外
    if (b)
      wall[0][x - 1] |= (1 << y);
    else
      wall[0][x - 1] &= ~(1 << y);
    return;
  case Dir::South:
    if (x < 0 || x >= MAZE_SIZE || y < 1 || y >= MAZE_SIZE)
      return; //< 盤面外
    if (b)
      wall[1][y - 1] |= (1 << x);
    else
      wall[1][y - 1] &= ~(1 << x);
    return;
  }
  loge << "invalid direction" << std::endl;
}

} // namespace MazeLib
