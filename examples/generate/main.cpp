#include "MazeLib/CLRobotBase.h"
#include "MazeLib/Maze.h"
#include "MazeLib/StepMap.h"
#include "MazeLib/StepMapSlalom.h"

#include <algorithm>
#include <queue>
#include <random>
#include <stack>

using namespace MazeLib;

void poll(Maze& maze) {
  /* random generator */
  std::random_device seed_gen;
  std::mt19937 engine(seed_gen());
  const auto getRandomDirectionsAlong4 = [&]() {
    Directions dirs;
    for (const auto d : Direction::Along4)
      dirs.push_back(d);
    std::shuffle(dirs.begin(), dirs.end(), engine);
    return dirs;
  };
  /* prepare maze */
  maze.reset(true, true);
  for (int x = 0; x < MAZE_SIZE; ++x)
    for (int y = 0; y < MAZE_SIZE; ++y) {
      auto p = Position(x, y);
      if (p == maze.getStart())
        continue;
      auto dirs = getRandomDirectionsAlong4();
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

void dig(Maze& maze) {
  /* random generator */
  std::random_device seed_gen;
  std::mt19937 engine(seed_gen());
  const auto getRandomFloat = [&]() {
    return std::generate_canonical<float, std::numeric_limits<float>::digits>(
        engine);
  };
  // const auto getRandomDirectionsAlong4 = [&]() {
  //   Directions dirs;
  //   for (const auto d : Direction::getAlong4())
  //     dirs.push_back(d);
  //   std::shuffle(dirs.begin(), dirs.end(), engine);
  //   return dirs;
  // };
  /* prepare maze */
  maze.reset(true, true);
  for (uint16_t i = 0; i < WallIndex::SIZE; ++i)
    maze.setWall(WallIndex(i), true), maze.setKnown(WallIndex(i), true);
  /* start cell */
  maze.updateWall(Position(0, 0), Direction::North, false);
  maze.setKnown(Position(0, 0), Direction::North, true);
  /* depth first search */
  std::stack<Pose> stack;
  std::bitset<Position::SIZE> mark;
  mark[maze.getStart().getIndex()] = true;
  stack.push({maze.getStart().next(Direction::North), Direction::North});
  while (!stack.empty()) {
    /* pop */
    const auto pose = stack.top();
    stack.pop();
    const auto p = pose.p;
    mark[p.getIndex()] = true;
    /* print */
    // maze.print({p}), getc(stdin);
    /* walk */
    Directions dirs;
    for (const auto d : Direction::Along4)
      dirs.push_back(d);
#if 1
    for (int i = 0; i < 2; ++i)
      dirs.push_back(Direction::Front);
    if (!stack.empty()) {
      if (Direction(pose.d - stack.top().d) == Direction::Front)
        for (int i = 0; i < 4; ++i)
          dirs.push_back(Direction::Front);
      if (Direction(pose.d - stack.top().d) == Direction::Left)
        for (int i = 0; i < 8; ++i)
          dirs.push_back(Direction::Right);
      if (Direction(pose.d - stack.top().d) == Direction::Right)
        for (int i = 0; i < 8; ++i)
          dirs.push_back(Direction::Left);
    }
#endif
    std::shuffle(dirs.begin(), dirs.end(), engine);
    while (!dirs.empty()) {
      const auto d = dirs.back() + pose.d;
      dirs.pop_back();
      if (!p.next(d).isInsideOfField())
        continue;
      if (mark[p.next(d).getIndex()])
        continue;
      maze.setWall(pose.p, d, false);
      stack.push(pose);
      stack.push(pose.next(d));
#if 1
      const auto known = std::count_if(
          Direction::Along4.cbegin(), Direction::Along4.cend(),
          [&](const auto d) {
            const auto n = p.next(d);
            const auto next = n.next(d);
            return next.isInsideOfField() && !mark[next.getIndex()];
          });
      if (known == 0 && getRandomFloat() < 0.5f)
        maze.setWall(p.next(d), d, false);
#endif
      break;
    }
  }
}

void setGoalLongest(Maze& maze) {
  StepMap map;
  map.update(maze, {maze.getStart()}, false, false);
  StepMap::step_t maxStep = 0;
  Position max_position;
  for (int x = 0; x < MAZE_SIZE; ++x)
    for (int y = 0; y < MAZE_SIZE; ++y) {
      const auto p = Position(x, y);
      const auto s = map.getStep(p);
      if (s == StepMap::STEP_MAX || s <= maxStep)
        continue;
      maxStep = s;
      max_position = p;
    }
  maze.setGoals({max_position});
}

int main(void) {
  /* prepare maze */
  Maze maze;

  /* set walls */
  // poll(maze);
  dig(maze);

  /* post process */
  for (uint16_t i = 0; i < WallIndex::SIZE; ++i)
    maze.setKnown(WallIndex(i), true);

  /* set goal */
  setGoalLongest(maze);
  // maze.setGoals({{MAZE_SIZE - 1, MAZE_SIZE - 1}});
  // maze.setGoals({{MAZE_SIZE / 2, MAZE_SIZE / 2}});

  /* print */
  maze.print();
  std::ofstream of("gen.maze");
  maze.print(of);

  /* StepMapSlalom */
  const bool knownOnly = 0;
  StepMapSlalom map;
  StepMapSlalom::Indexes path;
  StepMapSlalom::EdgeCost edgeCost;
  map.update(maze, edgeCost,
             StepMapSlalom::convertDestinations(maze.getGoals()), knownOnly);
  map.genPathFromMap(path);
  const auto shortestDirections = map.indexes2directions(path);
  std::cout << std::endl;
  maze.print(shortestDirections);

#if 1
  /* Preparation */
  const auto pRobot = std::make_unique<CLRobotBase>(maze);
  CLRobotBase& robot = *pRobot;
  robot.replaceGoals(maze.getGoals());
  /* Search Run */
  robot.searchRun();
  /* Show Result */
  robot.printInfo(false);
  for (bool diagEnabled : {false, true}) {
    robot.calcShortestDirections(diagEnabled);
    std::cout << "Estimated Shortest Time "
              << (diagEnabled ? "(diag)" : "(no diag)") << ": "
              << robot.getSearchAlgorithm().getShortestCost() << "\t[ms]"
              << std::endl;
  }
#endif

  return 0;
}
