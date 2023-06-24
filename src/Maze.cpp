/**
 * @file Maze.cpp
 * @brief マイクロマウスの迷路クラスを定義
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2017-10-30
 * @copyright Copyright 2017 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include "MazeLib/Maze.h"

#include <algorithm>  //< for std::find, std::count_if
#include <iomanip>    //< for std::setw

namespace MazeLib {

/* Direction */
const std::array<Direction, 4> Direction::Along4 = {
    East,
    North,
    West,
    South,
};
const std::array<Direction, 4> Direction::Diag4 = {
    NorthEast,
    NorthWest,
    SouthWest,
    SouthEast,
};
std::ostream& operator<<(std::ostream& os, const Directions& obj) {
  for (const auto d : obj) os << d;
  return os;
}

/* Position */
Position Position::next(const Direction d) const {
  switch (d) {
    case Direction::East:
      return Position(x + 1, y);
    case Direction::NorthEast:
      return Position(x + 1, y + 1);
    case Direction::North:
      return Position(x, y + 1);
    case Direction::NorthWest:
      return Position(x - 1, y + 1);
    case Direction::West:
      return Position(x - 1, y);
    case Direction::SouthWest:
      return Position(x - 1, y - 1);
    case Direction::South:
      return Position(x, y - 1);
    case Direction::SouthEast:
      return Position(x + 1, y - 1);
    default:
      MAZE_LOGE << d << std::endl;
      return *this;
  }
}
Position Position::rotate(const Direction d) const {
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
      MAZE_LOGE << d << std::endl;
      return *this;
  }
}
std::ostream& operator<<(std::ostream& os, const Position p) {
  return os << "( " << std::setw(2) << (int)p.x << ", " << std::setw(2)
            << (int)p.y << ")";
}

/* Pose */
std::ostream& operator<<(std::ostream& os, const Pose& pose) {
  return os << "( " << std::setw(2) << (int)pose.p.x << ", " << std::setw(2)
            << (int)pose.p.y << ", " << pose.d.toChar() << ")";
}

/* WallIndex */
WallIndex WallIndex::next(const Direction d) const {
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
      MAZE_LOGE << d << std::endl;
      return WallIndex(x, y, z);
  }
}
std::ostream& operator<<(std::ostream& os, const WallIndex i) {
  return os << "( " << std::setw(2) << (int)i.x << ", " << std::setw(2)
            << (int)i.y << ", " << i.getDirection().toChar() << ")";
}

/* WallRecord */
std::ostream& operator<<(std::ostream& os, const WallRecord& obj) {
  return os << "( " << std::setw(2) << (int)obj.x << ", " << std::setw(2)
            << (int)obj.y << ", " << obj.getDirection().toChar() << ", "
            << (obj.b ? "true" : "false") << ")";
}

/* Maze */
void Maze::reset(const bool set_start_wall, const bool set_range_full) {
  wall.reset();
  known.reset();
  min_x = min_y = set_range_full ? 0 : (MAZE_SIZE - 1);
  max_x = max_y = set_range_full ? (MAZE_SIZE - 1) : 0;
  wallRecordsBackupCounter = 0;
  if (set_start_wall) {
    updateWall(Position(0, 0), Direction::East, true);    //< start cell
    updateWall(Position(0, 0), Direction::North, false);  //< start cell
  }
  wallRecords.clear();
}
int8_t Maze::wallCount(const Position p) const {
  const auto& dirs = Direction::Along4;
  return std::count_if(dirs.cbegin(), dirs.cend(),
                       [&](const Direction d) { return isWall(p, d); });
}
int8_t Maze::unknownCount(const Position p) const {
  const auto& dirs = Direction::Along4;
  return std::count_if(dirs.cbegin(), dirs.cend(),
                       [&](const Direction d) { return !isKnown(p, d); });
}
bool Maze::updateWall(const Position p, const Direction d, const bool b,
                      const bool pushRecords) {
  /* 既知の壁と食い違いがあったら未知壁としてreturn */
  if (isKnown(p, d) && isWall(p, d) != b) {
    setWall(p, d, false);
    setKnown(p, d, false);
    /* ログに追加 */
    if (pushRecords) wallRecords.push_back(WallRecord(p, d, b));
    return false;
  }
  /* 未知壁なら壁情報を更新 */
  if (!isKnown(p, d)) {
    setWall(p, d, b);
    setKnown(p, d, true);
    /* ログに追加 */
    if (pushRecords) wallRecords.push_back(WallRecord(p, d, b));
    /* 最大最小区画を更新 */
    min_x = std::min(p.x, min_x);
    min_y = std::min(p.y, min_y);
    max_x = std::max(p.x, max_x);
    max_y = std::max(p.y, max_y);
  }
  return true;
}
void Maze::resetLastWalls(const int num, const bool set_start_wall) {
  /* 直近の壁情報を削除 */
  for (int i = 0; i < num && !wallRecords.empty(); ++i) wallRecords.pop_back();
  /* 削除後の壁情報を取得 */
  const auto new_wallRecords = wallRecords;
  /* スタート壁を考慮して迷路を再構築 */
  reset(set_start_wall);
  for (const auto wr : new_wallRecords)
    updateWall(wr.getPosition(), wr.getDirection(), wr.b);
  return;
}
bool Maze::parse(std::istream& is) {
  /* determine the maze size */
  /* get file size */
  is.seekg(0, std::ios::end);  //< move the position to end
  const int file_size = 1 + is.tellg();
  is.seekg(0, std::ios::beg);  //< restore the position to begin
  /* estimated (minimum) file size [byte] : F = (4*M + 1 + 1) * (2*M + 1) */
  /* using quadratic formula, we have: M = (sqrt(2*F) - 2) / 4 */
  const int mazeSize = (std::sqrt(2 * file_size) - 2) / 4;
  if (mazeSize < 1) return false; /*< file size error */
  /* reset existing maze */
  reset(), goals.clear();
  char c;  //< temporal variable to use next
  for (int8_t y = mazeSize; y >= 0; --y) {
    /* vertiacal walls and cells */
    if (y != mazeSize) {
      is.ignore(10, '|');  //< skip until next '|'
      for (int8_t x = 0; x < mazeSize; ++x) {
        is.ignore(1);  //< skip a space
        c = is.get();
        if (c == 'S')
          start = Position(x, y);
        else if (c == 'G')
          goals.push_back(Position(x, y));
        is.ignore(1);  //< skip a space
        c = is.get();
        if (c == '|')
          Maze::updateWall(Position(x, y), Direction::East, true, false);
        else if (c == ' ')
          Maze::updateWall(Position(x, y), Direction::East, false, false);
      }
    }
    /* horizontal walls and pillars */
    for (uint8_t x = 0; x < mazeSize; ++x) {
      is >> c;  //< skip until next '+' or 'o'
      std::string s;
      for (int i = 0; i < 3; ++i) s += (char)is.get();
      if (s == "---")
        Maze::updateWall(Position(x, y), Direction::South, true, false);
      else if (s == "   ")
        Maze::updateWall(Position(x, y), Direction::South, false, false);
    }
  }
  return true;
}
bool Maze::parse(const std::vector<std::string>& data, const int mazeSize) {
  for (const auto xr : {true, false}) {
    for (const auto yr : {false, true}) {
      for (const auto xy : {false, true}) {
        for (const auto b0 : Direction::Along4) {
          for (const auto b1 : Direction::Along4) {
            for (const auto b2 : Direction::Along4) {
              for (const auto b3 : Direction::Along4) {
                const std::array<Direction, 4> bit_to_dir_map{{b0, b1, b2, b3}};
                reset(false);
                int diffs = 0;
                for (int8_t y = 0; y < mazeSize; ++y) {
                  for (int8_t x = 0; x < mazeSize; ++x) {
                    const int8_t xd = xr ? x : (mazeSize - x - 1);
                    const int8_t yd = yr ? y : (mazeSize - y - 1);
                    const signed char c = xy ? data[xd][yd] : data[yd][xd];
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
            }
          }
        }
      }
    }
  }
  return false;
}
void Maze::print(std::ostream& os, const int mazeSize) const {
  for (int8_t y = mazeSize; y >= 0; --y) {
    if (y != mazeSize) {
      os << '|';
      for (int8_t x = 0; x < mazeSize; ++x) {
        const auto p = Position(x, y);
        if (p == start)
          os << " S ";
        else if (std::find(goals.cbegin(), goals.cend(), p) != goals.cend())
          os << " G ";
        else
          os << "   ";
        const auto k = isKnown(x, y, Direction::East);
        const auto w = isWall(x, y, Direction::East);
        os << (k ? (w ? '|' : ' ') : '.');
      }
      os << std::endl;
    }
    for (int8_t x = 0; x < mazeSize; ++x) {
      const auto k = isKnown(x, y, Direction::South);
      const auto w = isWall(x, y, Direction::South);
      os << '+' << (k ? (w ? "---" : "   ") : " . ");
    }
    os << '+' << std::endl;
  }
}
void Maze::print(const Directions& dirs, const Position start, std::ostream& os,
                 const size_t mazeSize) const {
  /* preparation */
  std::vector<Pose> path;
  {
    Position p = start;
    for (const auto d : dirs) path.push_back({p, d}), p = p.next(d);
  }
  const auto& maze = *this;
  /* start to draw maze */
  for (int8_t y = mazeSize; y >= 0; --y) {
    if (y != (int)mazeSize) {
      for (uint8_t x = 0; x <= mazeSize; ++x) {
        /* Vertical Wall */
        const auto it =
            std::find_if(path.cbegin(), path.cend(), [&](const Pose& pose) {
              return WallIndex(pose.p, pose.d) ==
                     WallIndex(Position(x, y), Direction::West);
            });
        const auto w = maze.isWall(x, y, Direction::West);
        const auto k = maze.isKnown(x, y, Direction::West);
        if (it != path.cend())
          os << C_YE << it->d << C_NO;
        else
          os << (k ? (w ? "|" : " ") : (C_RE "." C_NO));
        /* Breaking Condition */
        if (x == mazeSize) break;
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
    for (uint8_t x = 0; x < mazeSize; ++x) {
      /* Pillar */
      os << '+';
      /* Horizontal Wall */
      const auto it =
          std::find_if(path.cbegin(), path.cend(), [&](const Pose pose) {
            return WallIndex(pose.p, pose.d) ==
                   WallIndex(Position(x, y), Direction::South);
          });
      const auto w = maze.isWall(x, y, Direction::South);
      const auto k = maze.isKnown(x, y, Direction::South);
      if (it != path.cend())
        os << C_YE << ' ' << it->d << ' ' << C_NO;
      else
        os << (k ? (w ? "---" : "   ") : (C_RE " . " C_NO));
    }
    /* Last Pillar */
    os << '+' << std::endl;
  }
}
void Maze::print(const Positions& positions, std::ostream& os,
                 const size_t mazeSize) const {
  /* preparation */
  const auto exists = [&](const Position p) {
    return std::find(positions.cbegin(), positions.cend(), p) !=
           positions.cend();
  };
  const auto& maze = *this;
  /* start to draw maze */
  for (int8_t y = mazeSize; y >= 0; --y) {
    if (y != (int)mazeSize) {
      for (uint8_t x = 0; x <= mazeSize; ++x) {
        /* Vertical Wall */
        const auto w = maze.isWall(x, y, Direction::West);
        const auto k = maze.isKnown(x, y, Direction::West);
        os << (k ? (w ? "|" : " ") : (C_RE "." C_NO));
        /* Breaking Condition */
        if (x == mazeSize) break;
        /* Cell */
        const auto p = Position(x, y);
        if (p == start)
          os << C_BL << " S " << C_NO;
        else if (std::find(goals.cbegin(), goals.cend(), p) != goals.cend())
          os << C_BL << " G " << C_NO;
        else if (exists(p))
          os << C_YE << " X " << C_NO;
        else
          os << "   ";
      }
      os << std::endl;
    }
    for (uint8_t x = 0; x < mazeSize; ++x) {
      /* Pillar */
      os << '+';
      /* Horizontal Wall */
      const auto w = maze.isWall(x, y, Direction::South);
      const auto k = maze.isKnown(x, y, Direction::South);
      os << (k ? (w ? "---" : "   ") : (C_RE " . " C_NO));
    }
    /* Last Pillar */
    os << '+' << std::endl;
  }
}
bool Maze::backupWallRecordsToFile(const std::string& filepath,
                                   const bool clear) {
  /* 変更なし */
  if (!clear && wallRecordsBackupCounter == wallRecords.size()) return true;
  /* 前のデータが残っていたら削除 */
  std::ifstream ifs(filepath, std::ios::ate);
  const auto size = static_cast<size_t>(ifs.tellg());
  ifs.close();
  if (clear || size / sizeof(WallRecord) > wallRecordsBackupCounter) {
    std::remove(filepath.c_str());
    wallRecordsBackupCounter = 0;
  }
  /* WallRecords を追記 */
  std::ofstream ofs(filepath, std::ios::binary | std::ios::app);
  if (ofs.fail()) {
    MAZE_LOGW << "failed to open file! " << filepath << std::endl;
    return false;
  }
  while (wallRecordsBackupCounter < wallRecords.size()) {
    const auto& wr = wallRecords[wallRecordsBackupCounter];
    ofs.write(reinterpret_cast<const char*>(&wr), sizeof(wr));
    wallRecordsBackupCounter++;
  }
  return true;
}
bool Maze::restoreWallRecordsFromFile(const std::string& filepath) {
  std::ifstream f(filepath, std::ios::binary);
  if (f.fail()) {
    MAZE_LOGW << "failed to open file! " << filepath << std::endl;
    return false;
  }
  reset();
  while (!f.eof()) {
    WallRecord wr;
    f.read(reinterpret_cast<char*>(&wr), sizeof(WallRecord));
    Position p = Position(wr.x, wr.y);
    Direction d = Direction(wr.d);
    bool b = wr.b;
    updateWall(p, d, b);
    wallRecordsBackupCounter++;
  }
  return true;
}

}  // namespace MazeLib
