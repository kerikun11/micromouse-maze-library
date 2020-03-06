#include <algorithm>
#include <random>
#include <stack>

#include "Maze.h"
#include "StepMap.h"
#include "StepMapSlalom.h"

using namespace MazeLib;

void poll(Maze &maze) {
  maze.reset(true, true);
  std::random_device seed_gen;
  std::mt19937 engine(seed_gen());
  for (int x = 0; x < MAZE_SIZE; ++x)
    for (int y = 0; y < MAZE_SIZE; ++y) {
      auto p = Position(x, y);
      if (p == maze.getStart())
        continue;

      Directions dirs;
      for (const auto d : Direction::getAlong4())
        dirs.push_back(d);
      std::shuffle(dirs.begin(), dirs.end(), engine);

      while (!dirs.empty()) {
        const auto d = dirs.back();
        dirs.pop_back();
        const auto i = WallIndex((d & 4) ? p.next(Direction::NorthEast) : p, d);
        if (!i.isInsideOfField())
          break;
        if (maze.isWall(i))
          continue;
        maze.setWall(i, true);
        break;
      }
    }
}

void dig(Maze &maze) {
  /* random generator */
  std::random_device seed_gen;
  std::mt19937 engine(seed_gen());
  const auto getRandomDirectionsAlong4 = [&]() {
    Directions dirs;
    for (const auto d : Direction::getAlong4())
      dirs.push_back(d);
    std::shuffle(dirs.begin(), dirs.end(), engine);
    return dirs;
  };
  /* prepare maze */
  maze.reset(true, true);
  for (uint16_t i = 0; i < WallIndex::SIZE; ++i)
    maze.setWall(WallIndex(i), true);
  maze.setKnown(Position(0, 0), Direction::North, false);
  /* depth first search */
  std::stack<Position> stack;
  std::bitset<Position::SIZE> mark;
  stack.push(maze.getStart());
  while (!stack.empty()) {
    /* pop */
    const auto p = stack.top();
    stack.pop();
    mark[p.getIndex()] = true;
    /* print */
    // maze.print({p});
    // getc(stdin);
    /* walk */
    auto dirs = getRandomDirectionsAlong4();
    while (!dirs.empty()) {
      const auto d = dirs.back();
      dirs.pop_back();
      if (maze.isKnown(p, d))
        continue;
      if (mark[p.next(d).getIndex()])
        continue;
      maze.setWall(p, d, false);
      maze.setKnown(p, d, true);
      stack.push(p);
      stack.push(p.next(d));
      break;
    }
  }
}

int main(void) {
  /* prepare maze */
  Maze maze;

  /* set walls */
  dig(maze);

  /* find goal */
  {
    StepMap map;
    map.update(maze, {maze.getStart()}, false, false);
    StepMap::step_t max_step = 0;
    Position max_position;
    for (int x = 0; x < MAZE_SIZE; ++x)
      for (int y = 0; y < MAZE_SIZE; ++y) {
        const auto p = Position(x, y);
        const auto s = map.getStep(p);
        if (s == StepMap::STEP_MAX || s <= max_step)
          continue;
        max_step = s;
        max_position = p;
      }
    maze.setGoals({max_position});
  }

  /* post process */
  for (uint16_t i = 0; i < WallIndex::SIZE; ++i)
    maze.setKnown(WallIndex(i), true);
  // maze.setGoals({{MAZE_SIZE - 1, MAZE_SIZE - 1}});
  maze.print();

  /* StepMapSlalom */
  for (const auto diag_enabled : {false, true}) {
    const bool known_only = 0;
    StepMapSlalom map;
    StepMapSlalom::Indexes path;
    StepMapSlalom::EdgeCost edge_cost;
    map.update(maze, edge_cost,
               StepMapSlalom::convertDestinations(maze.getGoals()), known_only,
               diag_enabled);
    map.genPathFromMap(path);
    const auto shortest_dirs = map.indexes2directions(path, diag_enabled);
    std::cout << std::endl;
    maze.print(shortest_dirs);
  }
  return 0;
}
