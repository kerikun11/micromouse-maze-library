#include "MazeLib/CLRobotBase.h"

#define SEARCH_RUN_ENABLED 1
#define POSITION_IDENTIFICATION_RUN_ENABLED 1
#define STEP_MAP_ENABLED 1
#define STEP_MAP_WALL_ENABLED 1
#define STEP_MAP_SLALOM_ENABLED 1
#define SHOW_MAZE 0
#define SHOW_OBJECT_SIZE 0

using namespace MazeLib;

class CLRobot : public CLRobotBase {
 public:
  CLRobot(Maze& maze_target) : CLRobotBase(maze_target) {}
};

int test_meas(const std::string& mazedata_dir = "../mazedata/data/",
              const std::string& save_dir = "./") {
#if SHOW_OBJECT_SIZE
  /* show size */
  maze_logi << "sizeof(Maze):\t" << sizeof(Maze) << std::endl;
  maze_logi << "sizeof(StepMap):\t" << sizeof(StepMap) << std::endl;
  maze_logi << "sizeof(StepMapWall):\t" << sizeof(StepMapWall) << std::endl;
  maze_logi << "sizeof(StepMapSlalom):\t" << sizeof(StepMapSlalom) << std::endl;
  maze_logi << "sizeof(SearchAlgorithm):\t" << sizeof(SearchAlgorithm)
            << std::endl;
  maze_logi << "sizeof(RobotBase):\t" << sizeof(RobotBase) << std::endl;
#endif

  /* save file */
  std::ofstream csv(save_dir + "measurement.csv");
  /* print header */
  csv << "name\tsearch_time\tsearch_time_ms\tstep\tstep_f\tstep_l\tstep_"
         "r\tstep_b\twalls\tcalc_time_max\tshortest_ms_a\tshortest_ms_d"
#if POSITION_IDENTIFICATION_RUN_ENABLED
         "\tpi_calc_time_max\tpi_time_min\tpi_time_max\t"
         "pi_walls_min\tpi_walls_max"
#endif
      << std::endl;
  /* queue test files */
  std::vector<std::string> names;
#if 0
  names.push_back("32MM2021HX");
  // names.push_back("32MM2015HX");  // max calc time is longest in 32x32
  // names.push_back("16MM2014CX");  // max calc time is longest in 16x16
  // names.push_back("32_no_wall");
  // names.push_back("32_farm");
  // names.push_back("32_test_01");
  // names.push_back("32_DFS_01");
  // names.push_back("32_unknown");
  // names.push_back("32_fake");
#else
  for (int year = 2021; year >= 2008; --year)
    if (year != 2020)
      names.push_back("32MM" + std::to_string(year) + "HX");
#if 1
  for (int year = 2018; year >= 2014; --year)
    names.push_back("21MM" + std::to_string(year) + "HX_Taiwan");
  for (int year = 2020; year >= 2012; --year)
    names.push_back("16MM" + std::to_string(year) + "CX");
  for (int year = 2020; year >= 2017; --year)
    names.push_back("16MM" + std::to_string(year) + "H_student");
  for (int year = 2020; year >= 2017; --year)
    names.push_back("16MM" + std::to_string(year) + "C_student");
  for (int year = 2019; year >= 2017; --year)
    names.push_back("16MM" + std::to_string(year) + "H_Tashiro");
  for (int year = 2019; year >= 2017; --year)
    names.push_back("16MM" + std::to_string(year) + "H_Chubu");
  for (int year = 2019; year >= 2016; --year)
    names.push_back("16MM" + std::to_string(year) + "H_Kansai");
  for (int year = 2017; year >= 2015; --year)
    names.push_back("16MM" + std::to_string(year) + "C_Chubu");
  names.push_back("16MM2021H_semi");
  names.push_back("16MM2021H_Kansai");
  names.push_back("16MM2019H_semi");
  names.push_back("16MM2019H_Kyushu");
  names.push_back("16MM2019H_Kanazawa");
  names.push_back("16MM2019H_Hokuriku");
  names.push_back("16MM2019H_East");
  names.push_back("16MM2019H_Cheese");
  names.push_back("16MM2018H_semi");
  names.push_back("16MM2017HX_pre");
  names.push_back("16MM2017H_Cheese");
  names.push_back("16MM2017CX_pre");
  names.push_back("16MM2017C_East");
  names.push_back("16MM2016C_Kyushu");
  names.push_back("09MM2019C_Cheese");
  names.push_back("08MM2016CF_pre");
#endif
#endif

  /* test for each maze */
  for (const auto& name : names) {
    std::cout << std::endl;
    std::cout << "Maze: \t" << name << std::endl;
    csv << name;

    /* Maze Target */
    const auto p_maze_target = std::make_unique<Maze>();
    Maze& maze_target = *p_maze_target;
    if (!maze_target.parse(mazedata_dir + name + ".maze")) {
      maze_loge << "File Parse Error!" << std::endl;
      continue;
    }

#if SEARCH_RUN_ENABLED
    /* Search Run */
    const auto p_robot = std::make_unique<CLRobot>(maze_target);
    CLRobot& robot = *p_robot;
    robot.replaceGoals(maze_target.getGoals());
    if (!robot.searchRun())
      maze_loge << "Failed to Find a Path to Goal!" << std::endl;
    robot.printSearchResult();
    csv << "\t" << robot.cost / 1000 / 60 << ":" << std::setw(2)
        << std::setfill('0') << robot.cost / 1000 % 60;
    csv << "\t" << robot.cost << "\t" << robot.step << "\t" << robot.f << "\t"
        << robot.l << "\t" << robot.r << "\t" << robot.b;
    csv << "\t" << robot.getMaze().getWallRecords().size();
    std::cout << "Max Calc Time:\t" << robot.tCalcMax << "\t[us]" << std::endl;
    csv << "\t" << robot.tCalcMax;
    std::ofstream res(save_dir + "search-logs-" + name + ".csv");
    robot.printSearchLogs(res);
    /* FastRun */
    for (const auto diagEnabled : {false, true}) {
      if (!robot.calcShortestDirections(diagEnabled)) {
        maze_loge << "Failed to Find a Shortest Path! "
                  << (diagEnabled ? "diag" : "no_diag") << std::endl;
        continue;
      }
      const auto path_cost = robot.getSearchAlgorithm().getShortestCost();
      std::cout << "PathCost " << (diagEnabled ? "diag" : "no_d") << ":\t"
                << path_cost << "\t[ms]" << std::endl;
      csv << "\t" << path_cost;
      robot.fastRun(diagEnabled);
#if SHOW_MAZE
      robot.printPath();
#endif
      /* Shortest Path Comparison */
      const auto p_at = std::make_unique<Agent>(maze_target);
      Agent& at = *p_at;
      at.calcShortestDirections(diagEnabled);
      robot.calcShortestDirections(diagEnabled);
      if (at.getShortestDirections() != robot.getShortestDirections()) {
        maze_logw << "searched path is not shortest! "
                  << (diagEnabled ? "(diag)" : "(no_diag)") << std::endl;
        maze_logw << "real: " << at.getSearchAlgorithm().getShortestCost()
                  << " searched: "
                  << robot.getSearchAlgorithm().getShortestCost() << std::endl;
#if SHOW_MAZE
        at.printPath(), robot.printPath();
#endif
      }
    }
#if MAZE_DEBUG_PROFILING
    std::cout << "StepMap MaxQueue: "
              << robot.getSearchAlgorithm().getStepMap().queue_size_max
              << std::endl;
    std::cout << "StepMapWall     : "
              << robot.getSearchAlgorithm().getStepMapWall().queue_size_max
              << std::endl;
    std::cout << "StepMapSlalom   : "
              << robot.getSearchAlgorithm().getStepMapSlalom().queue_size_max
              << std::endl;
#endif
#endif

#if POSITION_IDENTIFICATION_RUN_ENABLED
    /* Position Identification Run */
    robot.tCalcMax = 0;
    uint32_t pi_time_max = 0;    /*< 探索時間 [秒] */
    uint32_t pi_time_min = 1e6f; /*< 探索時間 [秒] */
    const auto p_step_map = std::make_unique<StepMap>();
    StepMap& step_map = *p_step_map;
    const auto p_maze_pi = std::make_unique<Maze>();
    Maze& maze_pi = *p_maze_pi;
    maze_pi = robot.getMaze(); /*< 探索終了時の迷路を取得 */
    /* 迷路的に行き得る区画を洗い出す */
    step_map.update(maze_target, {maze_target.getStart()}, true, true);
    for (int8_t x = 0; x < MAZE_SIZE; ++x)
      for (int8_t y = 0; y < MAZE_SIZE; ++y)
        for (const auto d : Direction::Along4) {
          const auto p = Position(x, y);
          if (p == Position(0, 0))
            continue; /*< スタートは除外 */
          if (step_map.getStep(p) == StepMap::STEP_MAX)
            continue; /*< そもそも迷路的に行き得ない区画は除外 */
          if (maze_target.isWall(p, d + Direction::Back))
            continue; /*< 壁上からは除外 */
          robot.fake_offset = robot.real = Pose(p, d);
          robot.updateMaze(maze_pi); /*< 探索直後の迷路に置き換える */
          // robot.resetLastWalls(maze_pi.getWallRecords().size() / 5);
          robot.setForceGoingToGoal(); /*< ゴールへの訪問を指定 */
          const bool res = robot.positionIdentifyRun();
          if (!res)
            maze_loge << "Failed to Identify! fake_offset: "
                      << robot.fake_offset << std::endl;
          /* save result */
          pi_time_max = std::max(pi_time_max, robot.cost);
          pi_time_min = std::min(pi_time_min, robot.cost);
        }
    /* print result */
    std::cout << "P.I. tCalcMax:\t" << robot.tCalcMax << "\t[us]" << std::endl;
    std::cout << "P.I. tEst:\t" << (int(pi_time_min) / 60) % 60 << ":"
              << std::setw(2) << std::setfill('0') << int(pi_time_min) % 60
              << "\t" << pi_time_max / 1000 / 60 % 60 << ":" << std::setw(2)
              << std::setfill('0') << pi_time_max / 1000 % 60
              << std::setfill(' ') << std::endl;
    std::cout << "P.I. walls:\t" << robot.walls_pi_min << "\t"
              << robot.walls_pi_max << std::endl;
    csv << "\t" << robot.tCalcMax;
    csv << "\t" << pi_time_min;
    csv << "\t" << pi_time_max;
    csv << "\t" << robot.walls_pi_min;
    csv << "\t" << robot.walls_pi_max;
#endif

#if STEP_MAP_ENABLED
    /* StepMap */
    for (const auto simple : {true, false}) {
      const bool knownOnly = 0;
      const Maze& maze = maze_target;
      const auto p = std::make_unique<StepMap>();
      StepMap& map = *p;
      std::chrono::microseconds sum{0};
      const int n = 100;
      Directions shortestDirections;
      for (int i = 0; i < n; ++i) {
        const auto t_s = std::chrono::system_clock().now();
        shortestDirections =
            map.calcShortestDirections(maze, knownOnly, simple);
        if (shortestDirections.empty())
          maze_loge << "Failed!" << std::endl;
        const auto t_e = std::chrono::system_clock().now();
        const auto us =
            std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        sum += us;
      }
      std::cout << "StepMap " << (simple ? "simple" : "normal") << ":\t"
                << sum.count() / n << "\t[us]" << std::endl;
#if MAZE_DEBUG_PROFILING
      std::cout << "StepMap MaxQueue: " << map.queue_size_max << std::endl;
#endif
#if SHOW_MAZE
      map.print(maze, shortestDirections);
      map.printFull(maze, shortestDirections);
      // maze.print(shortestDirections);
#endif
    }
#endif

#if STEP_MAP_WALL_ENABLED
    /* StepMapWall */
    for (const auto simple : {true, false}) {
      const bool knownOnly = 0;
      const Maze& maze = maze_target;
      const auto p = std::make_unique<StepMapWall>();
      StepMapWall& map = *p;
      std::chrono::microseconds sum{0};
      const int n = 100;
      Directions shortestDirections;
      for (int i = 0; i < n; ++i) {
        const auto t_s = std::chrono::system_clock().now();
        shortestDirections =
            map.calcShortestDirections(maze, knownOnly, simple);
        if (shortestDirections.empty())
          maze_loge << "Failed!" << std::endl;
        const auto t_e = std::chrono::system_clock().now();
        const auto us =
            std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        sum += us;
      }
      std::cout << "StepMapWall " << (simple ? "s" : "n") << ":\t"
                << sum.count() / n << "\t[us]" << std::endl;
      StepMapWall::appendStraightDirections(maze, shortestDirections);
#if MAZE_DEBUG_PROFILING
      std::cout << "StepMapWall MaxQueue: " << map.queue_size_max << std::endl;
#endif
#if SHOW_MAZE
      map.print(maze, shortestDirections);
      map.print(maze, shortestDirections, StepMapWall::START_WALL_INDEX, true);
      map.printPath(maze, shortestDirections);
      maze.print(StepMapWall::convertWallIndexDirectionsToPositionDirections(
          shortestDirections));
#endif
    }
#endif

#if STEP_MAP_SLALOM_ENABLED
    /* StepMapSlalom */
    {
      const bool knownOnly = 0;
      const bool diagEnabled = 1;
      const Maze& maze = maze_target;
      const auto p = std::make_unique<StepMapSlalom>();
      StepMapSlalom& map = *p;
      std::chrono::microseconds sum{0};
      const int n = 100;
      StepMapSlalom::EdgeCost edgeCost;
      StepMapSlalom::Indexes path;
      Directions shortestDirections;
      for (int i = 0; i < n; ++i) {
        const auto t_s = std::chrono::system_clock().now();
        map.update(maze, edgeCost,
                   StepMapSlalom::convertDestinations(maze.getGoals()),
                   knownOnly);
        if (!map.genPathFromMap(path)) {
          maze_loge << "Failed!" << std::endl;
        }
        shortestDirections = map.indexes2directions(path);
        StepMap::appendStraightDirections(maze, shortestDirections, knownOnly,
                                          diagEnabled);
        const auto t_e = std::chrono::system_clock().now();
        const auto us =
            std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        sum += us;
      }
      std::cout << "StepMapSlalom:\t" << sum.count() / n << "\t[us]"
                << std::endl;
#if MAZE_DEBUG_PROFILING
      std::cout << "StepMapSlalom MaxQueue: " << map.queue_size_max
                << std::endl;
#endif
#if SHOW_MAZE
      map.printPath(maze, path);
      map.print(maze, path);
      maze.print(shortestDirections);
#endif
    }
#endif

#if 0
    std::ifstream f(save_dir + name + ".csv");
    std::string line;
    while (std::getline(f, line)) {
      std::cout << line << std::endl;
    }
#endif

    csv << std::endl;
  }
  std::cout << std::endl << "Measurement End" << std::endl;

#if 1
  std::ifstream f(save_dir + "measurement.csv");
  std::string line;
  while (std::getline(f, line)) {
    std::cout << line << std::endl;
  }
#endif

  return 0;
}

int main(void) {
  return test_meas();
}
