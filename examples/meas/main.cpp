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
  CLRobot(Maze& mazeTarget) : CLRobotBase(mazeTarget) {}
};

std::vector<std::string> getTargetMazeNames() {
  /* queue test files */
  std::vector<std::string> names;
#if 0
  names.push_back("32MM2022HX");
  // names.push_back("32MM2015HX");  // max calc time is longest in 32x32
  // names.push_back("16MM2014CX");  // max calc time is longest in 16x16
  // names.push_back("32_no_wall");
  // names.push_back("32_farm");
  // names.push_back("32_test_01");
  // names.push_back("32_DFS_01");
  // names.push_back("32_unknown");
  // names.push_back("32_fake");
#else
  for (int year = 2022; year >= 2008; --year)
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
  return names;
}

void show_obj_size() {
  MAZE_LOGI << "sizeof(Maze):\t" << sizeof(Maze) << std::endl;
  MAZE_LOGI << "sizeof(StepMap):\t" << sizeof(StepMap) << std::endl;
  MAZE_LOGI << "sizeof(StepMapWall):\t" << sizeof(StepMapWall) << std::endl;
  MAZE_LOGI << "sizeof(StepMapSlalom):\t" << sizeof(StepMapSlalom) << std::endl;
  MAZE_LOGI << "sizeof(SearchAlgorithm):\t" << sizeof(SearchAlgorithm)
            << std::endl;
  MAZE_LOGI << "sizeof(RobotBase):\t" << sizeof(RobotBase) << std::endl;
}

int test_meas(const std::string& mazedata_dir = "../mazedata/data/",
              const std::string& save_dir = "./") {
#if SHOW_OBJECT_SIZE
  /* show size */
  show_obj_size();
#endif

  /* prepare output csv file */
  std::ofstream csv(save_dir + "measurement.csv");
  /* print csv header */
  csv << "name\tsearch_time\tsearch_time_ms\tstep\tstep_f\tstep_l\tstep_"
         "r\tstep_b\twalls\tcalc_time_max\tshortest_ms_a\tshortest_ms_d"
#if POSITION_IDENTIFICATION_RUN_ENABLED
         "\tpi_calc_time_max\tpi_est_time_min\tpi_est_time_max\t"
         "pi_walls_min\tpi_walls_max"
#endif
      << std::endl;

  /* test for each maze */
  const auto names = getTargetMazeNames();
  for (const auto& name : names) {
    std::cout << std::endl;
    std::cout << "Maze: \t" << name << std::endl;
    csv << name;

    /* Maze Target */
    const auto pMazeTarget = std::make_unique<Maze>();
    Maze& mazeTarget = *pMazeTarget;
    const auto filename = mazedata_dir + name + ".maze";
    if (!mazeTarget.parse(filename)) {
      MAZE_LOGE << "File Parse Error! " << filename << std::endl;
      continue;
    }

#if SEARCH_RUN_ENABLED
    /* Search Run */
    const auto pRobot = std::make_unique<CLRobot>(mazeTarget);
    CLRobot& robot = *pRobot;
    robot.replaceGoals(mazeTarget.getGoals());
    if (!robot.searchRun())
      MAZE_LOGE << "Failed to Find a Path to Goal!" << std::endl;
    robot.printSearchResult();
    csv << "\t" << robot.est_time_ms / 1000 / 60 << ":" << std::setw(2)
        << std::setfill('0') << robot.est_time_ms / 1000 % 60;
    csv << "\t" << robot.est_time_ms << "\t" << robot.step << "\t" << robot.f
        << "\t" << robot.l << "\t" << robot.r << "\t" << robot.b;
    csv << "\t" << robot.getMaze().getWallRecords().size();
    std::cout << "Max Calc Time:\t" << robot.calc_time_max << "\t[us]"
              << std::endl;
    csv << "\t" << robot.calc_time_max;
    std::ofstream res(save_dir + name + ".csv");
    robot.printSearchLogs(res);
    /* FastRun */
    for (const auto diagEnabled : {false, true}) {
      if (!robot.calcShortestDirections(diagEnabled)) {
        MAZE_LOGE << "Failed to Find a Shortest Path! "
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
      const auto pAgent = std::make_unique<Agent>();
      Agent& at = *pAgent;
      at.updateMaze(mazeTarget);
      at.calcShortestDirections(diagEnabled);
      robot.calcShortestDirections(diagEnabled);
      if (at.getSearchAlgorithm().getShortestCost() !=
          robot.getSearchAlgorithm().getShortestCost()) {
        MAZE_LOGW << "searched path is not shortest! "
                  << (diagEnabled ? "(diag)" : "(no_diag)") << std::endl;
        MAZE_LOGW << "real: " << at.getSearchAlgorithm().getShortestCost()
                  << " searched: "
                  << robot.getSearchAlgorithm().getShortestCost() << std::endl;
        // at.printPath(), robot.printPath();
      }
    }
#if MAZE_DEBUG_PROFILING
    std::cout << "StepMap MaxQueue: "
              << robot.getSearchAlgorithm().getStepMap().queueSizeMax
              << std::endl;
    std::cout << "StepMapWall     : "
              << robot.getSearchAlgorithm().getStepMapWall().queueSizeMax
              << std::endl;
    std::cout << "StepMapSlalom   : "
              << robot.getSearchAlgorithm().getStepMapSlalom().queueSizeMax
              << std::endl;
#endif
#endif

#if POSITION_IDENTIFICATION_RUN_ENABLED
    /* Position Identification Run */
    robot.calc_time_max = 0;
    robot.positionIdentifyRunForAllOffset();
    /* print result */
    std::cout << "P.I. Max Calc:\t" << robot.calc_time_max << "\t[us]"
              << std::endl;
    std::cout << "P.I. Est Time:\t" << robot.pi_est_time_ms_min / 1000 / 60 % 60
              << ":" << std::setw(2) << std::setfill('0')
              << robot.pi_est_time_ms_min / 1000 % 60 << "\t"
              << robot.pi_est_time_ms_max / 1000 / 60 % 60 << ":"
              << std::setw(2) << std::setfill('0')
              << robot.pi_est_time_ms_max / 1000 % 60 << std::setfill(' ')
              << std::endl;
    std::cout << "P.I. walls:\t" << robot.pi_walls_min << "\t"
              << robot.pi_walls_max << std::endl;
    csv << "\t" << robot.calc_time_max;
    csv << "\t" << robot.pi_est_time_ms_min;
    csv << "\t" << robot.pi_est_time_ms_max;
    csv << "\t" << robot.pi_walls_min;
    csv << "\t" << robot.pi_walls_max;
#endif

#if STEP_MAP_ENABLED
    /* StepMap */
    for (const auto simple : {true, false}) {
      const bool knownOnly = 0;
      const Maze& maze = mazeTarget;
      const auto p = std::make_unique<StepMap>();
      StepMap& map = *p;
      int sum = 0;
      const int n = 100;
      Directions shortestDirections;
      for (int i = 0; i < n; ++i) {
        const auto t_s = robot.microseconds();
        shortestDirections =
            map.calcShortestDirections(maze, knownOnly, simple);
        const auto t_e = robot.microseconds();
        if (shortestDirections.empty())
          MAZE_LOGE << "Failed!" << std::endl;
        sum += t_e - t_s;
      }
      std::cout << "StepMap " << (simple ? "simple" : "normal") << ":\t"
                << sum / n << "\t[us]" << std::endl;
#if MAZE_DEBUG_PROFILING
      std::cout << "StepMap MaxQueue: " << map.queueSizeMax << std::endl;
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
      const Maze& maze = mazeTarget;
      const auto p = std::make_unique<StepMapWall>();
      StepMapWall& map = *p;
      int sum = 0;
      const int n = 100;
      Directions shortestDirections;
      for (int i = 0; i < n; ++i) {
        const auto t_s = robot.microseconds();
        shortestDirections =
            map.calcShortestDirections(maze, knownOnly, simple);
        const auto t_e = robot.microseconds();
        if (shortestDirections.empty())
          MAZE_LOGE << "Failed!" << std::endl;
        sum += t_e - t_s;
      }
      std::cout << "StepMapWall " << (simple ? "s" : "n") << ":\t" << sum / n
                << "\t[us]" << std::endl;
      StepMapWall::appendStraightDirections(maze, shortestDirections);
#if MAZE_DEBUG_PROFILING
      std::cout << "StepMapWall MaxQueue: " << map.queueSizeMax << std::endl;
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
      const Maze& maze = mazeTarget;
      const auto p = std::make_unique<StepMapSlalom>();
      StepMapSlalom& map = *p;
      int sum = 0;
      const int n = 100;
      StepMapSlalom::EdgeCost edgeCost;
      StepMapSlalom::Indexes path;
      Directions shortestDirections;
      for (int i = 0; i < n; ++i) {
        const auto t_s = robot.microseconds();
        map.update(maze, edgeCost,
                   StepMapSlalom::convertDestinations(maze.getGoals()),
                   knownOnly);
        if (!map.genPathFromMap(path)) {
          MAZE_LOGE << "Failed!" << std::endl;
        }
        shortestDirections = map.indexes2directions(path);
        StepMap::appendStraightDirections(maze, shortestDirections, knownOnly,
                                          diagEnabled);
        const auto t_e = robot.microseconds();
        sum += t_e - t_s;
      }
      std::cout << "StepMapSlalom:\t" << sum / n << "\t[us]" << std::endl;
#if MAZE_DEBUG_PROFILING
      std::cout << "StepMapSlalom MaxQueue: " << map.queueSizeMax << std::endl;
#endif
#if SHOW_MAZE
      map.printPath(maze, path);
      map.print(maze, path);
      maze.print(shortestDirections);
#endif
    }
#endif

#if 0
    /* print csv file */
    std::ifstream f(save_dir + name + ".csv");
    std::string line;
    while (std::getline(f, line))
      std::cout << line << std::endl;
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
