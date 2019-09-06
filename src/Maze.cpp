/**
 * @file Maze.cpp
 * @brief マイクロマウスの迷路クラスを定義
 * @author KERI (Github: kerikun11)
 * @url https://kerikeri.top/
 * @date 2017.10.30
 */
#include "Maze.h"

#include <algorithm> //< for std::find(), std::count_if()
#include <iomanip>   //< for std::setw()

namespace MazeLib {

/* Position */
const Position Position::next(const Direction d) const {
  switch (d) {
  case Direction::East:
    return Position(x + 1, y);
  case Direction::North:
    return Position(x, y + 1);
  case Direction::West:
    return Position(x - 1, y);
  case Direction::South:
    return Position(x, y - 1);
  default:
    logw << "Invalid Direction: " << d << std::endl;
    return *this;
  }
}
const Position Position::rotate(const Direction d) const {
  switch (d) {
  case Direction::East:
    return Position(x, y);
  case Direction::North:
    return Position(-y, x);
  case Direction::West:
    return Position(-x, -y);
  case Direction::South:
    return Position(y, -x);
  default:
    logw << "Invalid Direction: " << d << std::endl;
    return *this;
  }
}
std::ostream &operator<<(std::ostream &os, const Position p) {
  return os << "(" << std::setw(2) << (int)p.x << ", " << std::setw(2)
            << (int)p.y << ")";
}

/* Pose */
std::ostream &operator<<(std::ostream &os, const Pose &obj) {
  return os << "( " << std::setw(2) << (int)obj.first.x << ", " << std::setw(2)
            << (int)obj.first.y << ", " << obj.second.toChar() << ")";
}

/* WallIndex */
const WallIndex WallIndex::next(const Direction d) const {
  switch (d) {
  case Direction::East:
    return WallIndex(x + 1, y, z);
  case Direction::NorthEast:
    return WallIndex(x + (1 - z), y + z, 1 - z);
  case Direction::North:
    return WallIndex(x, y + 1, z);
  case Direction::NorthWest:
    return WallIndex(x - z, y + z, 1 - z);
  case Direction::West:
    return WallIndex(x - 1, y, z);
  case Direction::SouthWest:
    return WallIndex(x - z, y - (1 - z), 1 - z);
  case Direction::South:
    return WallIndex(x, y - 1, z);
  case Direction::SouthEast:
    return WallIndex(x + (1 - z), y - (1 - z), 1 - z);
  default:
    logw << "Invalid Direction: " << d << std::endl;
    return WallIndex(x, y, z);
  }
}
std::ostream &operator<<(std::ostream &os, const WallIndex i) {
  return os << "( " << std::setw(2) << (int)i.x << ", " << std::setw(2)
            << (int)i.y << ", " << i.getDirection().toChar() << ")";
}

/* WallLog */
std::ostream &operator<<(std::ostream &os, const WallLog obj) {
  return os << "( " << std::setw(2) << (int)obj.x << ", " << std::setw(2)
            << (int)obj.y << ", " << obj.getDirection().toChar() << ", "
            << (obj.b ? "true" : "false") << ")";
}

/* Maze */
void Maze::reset(const bool set_start_wall) {
  wall.reset();
  known.reset();
  min_x = MAZE_SIZE - 1;
  min_y = MAZE_SIZE - 1;
  max_x = 0;
  max_y = 0;
  backup_counter = 0;
  if (set_start_wall) {
    updateWall(Position(0, 0), Direction::East, true);   //< start cell
    updateWall(Position(0, 0), Direction::North, false); //< start cell
  }
  wallLogs.clear();
}
int8_t Maze::wallCount(const Position p) const {
  auto dirs = Direction::getAlong4();
  return std::count_if(dirs.cbegin(), dirs.cend(),
                       [&](const Direction d) { return isWall(p, d); });
}
int8_t Maze::unknownCount(const Position p) const {
  const auto dirs = Direction::getAlong4();
  return std::count_if(dirs.cbegin(), dirs.cend(),
                       [&](const Direction d) { return !isKnown(p, d); });
}
bool Maze::updateWall(const Position p, const Direction d, const bool b,
                      const bool pushLog) {
  /* 既知の壁と食い違いがあったら未知壁としてreturn */
  if (isKnown(p, d) && isWall(p, d) != b) {
    setWall(p, d, false);
    setKnown(p, d, false);
    /* ログに追加 */
    if (pushLog)
      wallLogs.push_back(WallLog(p, d, b));
    return false;
  }
  /* 未知壁なら壁情報を更新 */
  if (!isKnown(p, d)) {
    setWall(p, d, b);
    setKnown(p, d, true);
    /* ログに追加 */
    if (pushLog)
      wallLogs.push_back(WallLog(p, d, b));
    /* 最大最小区画を更新 */
    min_x = std::min(p.x, min_x);
    min_y = std::min(p.y, min_y);
    max_x = std::max(p.x, max_x);
    max_y = std::max(p.y, max_y);
  }
  return true;
}
void Maze::resetLastWall(const int num) {
  for (int i = 0; i < num; ++i) {
    if (wallLogs.empty())
      return;
    const auto wl = wallLogs.back();
    const auto wl_p = wl.getPosition();
    setWall(wl_p, wl.d, false);
    setKnown(wl_p, wl.d, false);
    wallLogs.pop_back();
  }
  return;
}
bool Maze::parse(std::istream &is) {
  is.seekg(0, std::ios_base::end);
  int file_size = is.tellg();
  is.seekg(0, std::ios_base::beg);
  /* 迷路サイズの決定 */
  int maze_size = 0;
  if (file_size > 8000)
    maze_size = 32;
  else if (file_size > 2000)
    maze_size = 16;
  else
    maze_size = 8;
  /* 初期化 */
  reset();
  goals.clear();
  for (int8_t y = maze_size; y >= 0; --y) {
    if (y != maze_size) {
      is.ignore(10, '|'); //< 次の|が出てくるまでスキップ
      for (int8_t x = 0; x < maze_size; ++x) {
        is.ignore(1); //< " " 空欄分をスキップ
        char c = is.get();
        if (c == 'S')
          start = Position(x, y);
        else if (c == 'G')
          goals.push_back(Position(x, y));
        is.ignore(1); //< " " 空欄分をスキップ
        c = is.get();
        if (c == '|')
          Maze::updateWall(Position(x, y), Direction::East, true);
        else if (c == ' ')
          Maze::updateWall(Position(x, y), Direction::East, false);
      }
    }
    for (uint8_t x = 0; x < maze_size; ++x) {
      is.ignore(10, '+'); //< 次の+が出てくるまでスキップ
      std::string s;
      s += (char)is.get();
      s += (char)is.get();
      s += (char)is.get();
      if (s == "---")
        Maze::updateWall(Position(x, y), Direction::South, true);
      else if (s == "   ")
        Maze::updateWall(Position(x, y), Direction::South, false);
    }
  }
  return true;
}
bool Maze::parse(const std::vector<std::string> data, const int maze_size) {
  for (const auto xr : {true, false})
    for (const auto yr : {true, false})
      for (const auto xy : {true, false})
        for (const auto b0 : Direction::getAlong4())
          for (const auto b1 : Direction::getAlong4())
            for (const auto b2 : Direction::getAlong4())
              for (const auto b3 : Direction::getAlong4()) {
                const std::array<Direction, 4> bit_to_dir_map{{b0, b1, b2, b3}};
                reset(false);
                int diffs = 0;
                for (int8_t y = 0; y < maze_size; ++y) {
                  for (int8_t x = 0; x < maze_size; ++x) {
                    const int8_t xd = xr ? x : (maze_size - x - 1);
                    const int8_t yd = yr ? y : (maze_size - y - 1);
                    const char c = xy ? data[xd][yd] : data[yd][xd];
                    uint8_t h = 0;
                    if ('0' <= c && c <= '9')
                      h = c - '0';
                    else if ('a' <= c && c <= 'f')
                      h = c - 'a' + 10;
                    else if ('A' <= c && c <= 'F')
                      h = c - 'A' + 10;
                    else if (0 <= c && c <= 15)
                      h = c;
                    if (!updateWall(Position(x, y), bit_to_dir_map[0], h & 0x01,
                                    false))
                      ++diffs;
                    if (!updateWall(Position(x, y), bit_to_dir_map[1], h & 0x02,
                                    false))
                      ++diffs;
                    if (!updateWall(Position(x, y), bit_to_dir_map[2], h & 0x04,
                                    false))
                      ++diffs;
                    if (!updateWall(Position(x, y), bit_to_dir_map[3], h & 0x08,
                                    false))
                      ++diffs;
                  }
                }
                if (diffs < MAZE_SIZE && isWall(0, 0, Direction::East) &&
                    !isWall(0, 0, Direction::North))
                  return true;
              }
  return false;
}
void Maze::print(std::ostream &os, const int maze_size) const {
  for (int8_t y = maze_size; y >= 0; --y) {
    if (y != maze_size) {
      os << '|';
      for (int8_t x = 0; x < maze_size; ++x) {
        const auto p = Position(x, y);
        if (p == start)
          os << " S ";
        else if (std::find(goals.cbegin(), goals.cend(), p) != goals.cend())
          os << " G ";
        else
          os << "   ";
        os << (isKnown(x, y, Direction::East)
                   ? (isWall(x, y, Direction::East) ? "|" : " ")
                   : ".");
      }
      os << std::endl;
    }
    for (int8_t x = 0; x < maze_size; ++x)
      os << "+"
         << (isKnown(x, y, Direction::South)
                 ? (isWall(x, y, Direction::South) ? "---" : "   ")
                 : " . ");
    os << "+" << std::endl;
  }
}
void Maze::print(const Directions &dirs, const Position start, std::ostream &os,
                 const size_t maze_size) const {
  /* preparation */
  std::vector<Pose> path;
  Position p = start;
  for (const auto d : dirs)
    path.push_back({p, d}), p = p.next(d);
  const auto &maze = *this;
  /* start to draw maze */
  os << std::endl;
  for (int8_t y = maze_size; y >= 0; --y) {
    if (y != (int)maze_size) {
      for (uint8_t x = 0; x <= maze_size; ++x) {
        /* Vertical Wall */
        const auto it =
            std::find_if(path.cbegin(), path.cend(), [&](const Pose pose) {
              return WallIndex(pose.first, pose.second) ==
                     WallIndex(Position(x, y), Direction::West);
            });
        const auto w = maze.isWall(x, y, Direction::West);
        const auto k = maze.isKnown(x, y, Direction::West);
        if (it != path.cend())
          os << C_YE << it->second << C_NO;
        else
          os << (k ? (w ? "|" : " ") : (C_RE "." C_NO));
        /* Breaking Condition */
        if (x == maze_size)
          break;
        /* Cell */
        const auto p = Position(x, y);
        if (p == start)
          os << C_BL << " S " << C_NO;
        else if (std::find(goals.cbegin(), goals.cend(), p) != goals.cend())
          os << C_BL << " G " << C_NO;
        else
          os << "   ";
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < maze_size; ++x) {
      /* Pillar */
      os << "+";
      /* Horizontal Wall */
      const auto it =
          std::find_if(path.cbegin(), path.cend(), [&](const Pose pose) {
            return WallIndex(pose.first, pose.second) ==
                   WallIndex(Position(x, y), Direction::South);
          });
      const auto w = maze.isWall(x, y, Direction::South);
      const auto k = maze.isKnown(x, y, Direction::South);
      if (it != path.cend())
        os << C_YE << " " << it->second << " " << C_NO;
      else
        os << (k ? (w ? "---" : "   ") : (C_RE " . " C_NO));
    }
    /* Last Pillar */
    os << "+" << std::endl;
  }
}
void Maze::appendStraightDirections(const Maze &maze, Directions &shortest_dirs,
                                    const bool diag_enabled) {
  /* ゴール区画までたどる */
  auto p = maze.getStart();
  for (const auto d : shortest_dirs)
    p = p.next(d);
  if (shortest_dirs.size() < 2)
    return;
  auto prev_dir = shortest_dirs[shortest_dirs.size() - 1 - 1];
  auto dir = shortest_dirs[shortest_dirs.size() - 1];
  /* ゴール区画内を行けるところまで直進(斜め考慮)する */
  bool loop = true;
  while (loop) {
    loop = false;
    // 斜めを考慮した進行方向を列挙する
    Directions dirs;
    const auto rel_dir = Direction(dir - prev_dir);
    if (diag_enabled && rel_dir == Direction::Left)
      dirs = {Direction(dir + Direction::Right), dir};
    else if (diag_enabled && rel_dir == Direction::Right)
      dirs = {Direction(dir + Direction::Left), dir};
    else
      dirs = {dir};
    // 行ける方向に行く
    for (const auto d : dirs) {
      if (maze.canGo(p, d)) {
        shortest_dirs.push_back(d);
        p = p.next(d);
        prev_dir = dir;
        dir = d;
        loop = true;
        break;
      }
    }
  }
}

} // namespace MazeLib
