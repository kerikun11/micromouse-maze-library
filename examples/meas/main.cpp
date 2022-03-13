#include "MazeLib/CLRobotBase.h"

#define PI_ENABLED 1

using namespace MazeLib;

static std::string save_dir = "./";
// static std::string save_dir = "/spiffs/";

class CLRobot : public CLRobotBase {
public:
  CLRobot(Maze &maze_target, const std::string &name)
      : CLRobotBase(maze_target), name(name) {
    csv.open(save_dir + name + ".csv", std::ios::out);
  }

protected:
  virtual void queueAction(const SearchAction action) override {
#if 1
    if (getState() == SearchAlgorithm::IDENTIFYING_POSITION &&
        real.p == maze.getStart() && action != ST_HALF_STOP)
      maze_logw << "Visited Start! fake_offset: " << fake_offset << std::endl;
#endif
    CLRobotBase::queueAction(action);
  }
  virtual void calcNextDirectionsPostCallback(
      SearchAlgorithm::State prevState __attribute__((unused)),
      SearchAlgorithm::State newState __attribute__((unused))) override {
    CLRobotBase::calcNextDirectionsPostCallback(prevState, newState);
    csv << t_dur << std::endl;
  }

private:
  std::string name;
  std::ofstream csv;
};

int test_meas(const std::string &mazedata_dir = "../mazedata/data/") {
  /* save file */
  std::ofstream csv(save_dir + "measurement.csv");
  // std::stringstream csv;
  csv << "name\tsearch_time\tcost_s\tstep\tstep_f\tstep_l\tstep_r\tstep_"
         "b\twalls\t"
         "calc_dur_max\tdur_search\tshortest_time_ms_along\tshortest_time_ms_"
         "diag"
#if PI_ENABLED
         "\tpi_calc_dur_max\tpi_cost_min\tpi_cost_max\tpi_walls_min\tpi_walls_"
         "max"
#endif
      << std::endl;
  /* queue test files */
  std::vector<std::string> names;
#if 0
  // names.push_back("32_unknown");
  // names.push_back("32MM2019HX");
  // names.push_back("32MM2015HX"); // max calc time is longest in 32x32
  names.push_back("16MM2014CX"); // max calc time is longest in 16x16
#else
  for (int year = 2021; year >= 2021; --year)
    names.push_back("32MM" + std::to_string(year) + "HX");
  for (int year = 2019; year >= 2010; --year)
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
  for (const auto name : {
           "16MM2021H_semi",
           "16MM2021H_Kansai",
           "16MM2019H_semi",
           "16MM2019H_Kyushu",
           "16MM2019H_Kanazawa",
           "16MM2019H_Hokuriku",
           "16MM2019H_East",
           "16MM2019H_Cheese",
           "16MM2018H_semi",
           "16MM2017HX_pre",
           "16MM2017H_Cheese",
           "16MM2017CX_pre",
           "16MM2017C_East",
           "16MM2016C_Kyushu",
           "09MM2019C_Cheese",
           "08MM2016CF_pre",
       })
    names.push_back(name);
#endif
#if 0
  for (const auto name : {
           "04_test",
           "32_fake",
           "32_no_wall",
           "32_unknown",
       })
    names.push_back(name);
#endif
#endif
  /* analyze for each maze */
  for (const auto &name : names) {
    std::cout << std::endl;
    std::cout << "Maze: \t" << name << std::endl;
    csv << name;

    /* Maze Target */
    const auto p_maze_target = std::make_unique<Maze>();
    Maze &maze_target = *p_maze_target;
    if (!maze_target.parse(mazedata_dir + name + ".maze")) {
      maze_loge << "File Parse Error!" << std::endl;
      continue;
    }

#if 1
    /* Search Run */
    const auto p_robot = std::make_unique<CLRobot>(maze_target, name);
    CLRobot &robot = *p_robot;
    robot.replaceGoals(maze_target.getGoals());
    const auto t_s = std::chrono::system_clock().now();
    if (!robot.searchRun())
      maze_loge << "Failed to Find a Path to Goal!" << std::endl;
    const auto t_e = std::chrono::system_clock().now();
    const auto t_search =
        std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s)
            .count();
    robot.printSearchResult();
    csv << "\t" << int(robot.cost / 60) << ":" << std::setw(2)
        << std::setfill('0') << int(robot.cost) % 60;
    csv << "\t" << robot.cost << "\t" << robot.step << "\t" << robot.f << "\t"
        << robot.l << "\t" << robot.r << "\t" << robot.b;
    csv << "\t" << robot.getMaze().getWallRecords().size();
    std::cout << "Max Calc Time:\t" << robot.t_dur_max << "\t[us]" << std::endl;
    csv << "\t" << robot.t_dur_max;
    // std::cout << "Total Search:\t" << t_search << "\t[us]" << std::endl;
    csv << "\t" << t_search;
    for (const auto diag_enabled : {false, true}) {
      if (!robot.calcShortestDirections(diag_enabled)) {
        maze_loge << "Failed to Find a Shortest Path! "
                  << (diag_enabled ? "diag" : "no_diag") << std::endl;
        continue;
      }
      const auto path_cost = robot.getSearchAlgorithm().getShortestCost();
      // std::cout << "PathCost " << (diag_enabled ? "diag" : "no_d") << ":\t"
      //           << path_cost << "\t[ms]" << std::endl;
      csv << "\t" << path_cost;
      robot.fastRun(diag_enabled);
      // robot.printPath();
      /* Shortest Path Comparison */
      const auto p_at = std::make_unique<Agent>(maze_target);
      Agent &at = *p_at;
      at.calcShortestDirections(diag_enabled);
      robot.calcShortestDirections(diag_enabled);
      if (at.getSearchAlgorithm().getShortestCost() !=
          robot.getSearchAlgorithm().getShortestCost()) {
        maze_logw << "searched path is not shortest! "
                  << (diag_enabled ? "(diag)" : "(no_diag)") << std::endl;
        maze_logw << "real: " << at.getSearchAlgorithm().getShortestCost()
                  << " searched: "
                  << robot.getSearchAlgorithm().getShortestCost() << std::endl;
        // at.printPath(); robot.printPath();
      }
    }
#endif

#if PI_ENABLED
    /* Position Identification Run */
    robot.t_dur_max = 0;
    float pi_cost_max = 0;    /*< 探索時間 [秒] */
    float pi_cost_min = 1e6f; /*< 探索時間 [秒] */
    const auto p_step_map = std::make_unique<StepMap>();
    StepMap &step_map = *p_step_map;
    const auto p_maze_pi = std::make_unique<Maze>();
    Maze &maze_pi = *p_maze_pi;
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
          pi_cost_max = std::max(pi_cost_max, robot.cost);
          pi_cost_min = std::min(pi_cost_min, robot.cost);
        }
    /* print result */
    std::cout << "P.I. Max Calc:\t" << robot.t_dur_max << "\t[us]" << std::endl;
    std::cout << "P.I. Time:\t" << (int(pi_cost_min) / 60) % 60 << ":"
              << std::setw(2) << std::setfill('0') << int(pi_cost_min) % 60
              << "\t" << (int(pi_cost_max) / 60) % 60 << ":" << std::setw(2)
              << std::setfill('0') << int(pi_cost_max) % 60 << std::setfill(' ')
              << std::endl;
    std::cout << "P.I. wall:\t" << robot.walls_pi_min << "\t"
              << robot.walls_pi_max << std::endl;
    csv << "\t" << robot.t_dur_max;
    csv << "\t" << pi_cost_min;
    csv << "\t" << pi_cost_max;
    csv << "\t" << robot.walls_pi_min;
    csv << "\t" << robot.walls_pi_max;
#endif

#if 0
    /* StepMap */
    for (const auto simple : {true, false}) {
      const bool known_only = 0;
      const Maze &maze = maze_target;
      const auto p = std::make_unique<StepMap>();
      StepMap &map = *p;
      std::chrono::microseconds sum{0};
      const int n = 100;
      Directions shortest_dirs;
      for (int i = 0; i < n; ++i) {
        const auto t_s = std::chrono::system_clock().now();
        shortest_dirs = map.calcShortestDirections(maze, known_only, simple);
        if (shortest_dirs.empty())
          maze_loge << "Failed!" << std::endl;
        const auto t_e = std::chrono::system_clock().now();
        const auto us =
            std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        sum += us;
      }
      std::cout << "StepMap " << (simple ? "simple" : "normal") << ":\t"
                << sum.count() / n << "\t[us]" << std::endl;
      // map.print(maze, shortest_dirs);
    }
#endif

#if 0
    /* StepMapWall */
    for (const auto simple : {true, false}) {
      const bool known_only = 0;
      const Maze &maze = maze_target;
      const auto p = std::make_unique<StepMapWall>();
      StepMapWall &map = *p;
      std::chrono::microseconds sum{0};
      const int n = 100;
      Directions shortest_dirs;
      for (int i = 0; i < n; ++i) {
        const auto t_s = std::chrono::system_clock().now();
        shortest_dirs = map.calcShortestDirections(maze, known_only, simple);
        if (shortest_dirs.empty())
          maze_loge << "Failed!" << std::endl;
        const auto t_e = std::chrono::system_clock().now();
        const auto us =
            std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        sum += us;
      }
      std::cout << "StepMapWall " << (simple ? "s" : "n") << ":\t"
                << sum.count() / n << "\t[us]" << std::endl;
      // map.print(maze, shortest_dirs);
      // maze.print(
      //     StepMapWall::convertWallIndexDirectionsToPositionDirections(
      //         shortest_dirs, WallIndex(Position(0, 0), Direction::North)),
      //     maze.getStart());
    }
#endif

#if 0
    /* StepMapSlalom */
    for (const auto diag_enabled : {false, true}) {
      const bool known_only = 0;
      const Maze &maze = maze_target;
      const auto p = std::make_unique<StepMapSlalom>();
      StepMapSlalom &map = *p;
      std::chrono::microseconds sum{0};
      const int n = 100;
      StepMapSlalom::Indexes path;
      StepMapSlalom::EdgeCost edge_cost;
      for (int i = 0; i < n; ++i) {
        const auto t_s = std::chrono::system_clock().now();
        map.update(maze, edge_cost,
                   StepMapSlalom::convertDestinations(maze.getGoals()),
                   known_only, diag_enabled);
        map.genPathFromMap(path);
        const auto t_e = std::chrono::system_clock().now();
        const auto us =
            std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        sum += us;
      }
      std::cout << "StepSla " << (diag_enabled ? "diag" : "no_d") << ":\t"
                << sum.count() / n << "\t[us]" << std::endl;
      // map.print(maze, path);
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

#if 0
  std::cout << std::endl << csv.rdbuf() << std::endl;
#endif

  return 0;
}

int main(void) { return test_meas(); }
