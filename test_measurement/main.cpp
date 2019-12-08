#include "CLRobotBase.h"
#include "ShortestAlgorithm.h"

using namespace MazeLib;

class CLRobot : public CLRobotBase {
public:
  CLRobot(Maze &maze_target) : CLRobotBase(maze_target) {}

protected:
  virtual void queueAction(const Action action) override {
#if 1
    if (getState() == SearchAlgorithm::IDENTIFYING_POSITION &&
        real.p == maze.getStart() && action != ST_HALF_STOP)
      logw << "Visited Start! fake_offset: " << fake_offset << std::endl;
#endif
    CLRobotBase::queueAction(action);
  }
};

int test_measurement() {
  std::ofstream csv("measurement.csv");
  /* specify maze data directory */
  const std::string mazedata_dir = "../mazedata/";
  /* queue test files */
  std::vector<std::string> filenames;
#if 0
  // filenames.push_back("32_unknown.maze");
  // filenames.push_back("16MM2019H_Chubu.maze");
  filenames.push_back("32MM2016HX.maze");
#else
  for (int year = 2019; year >= 2010; --year)
    filenames.push_back("32MM" + std::to_string(year) + "HX.maze");
  for (int year = 2018; year >= 2014; --year)
    filenames.push_back("21MM" + std::to_string(year) + "HX_Taiwan.maze");
  for (int year = 2018; year >= 2012; --year)
    filenames.push_back("16MM" + std::to_string(year) + "CX.maze");
  for (int year = 2017; year >= 2015; --year)
    filenames.push_back("16MM" + std::to_string(year) + "C_Chubu.maze");
  for (int year = 2019; year >= 2017; --year)
    filenames.push_back("16MM" + std::to_string(year) + "H_Chubu.maze");
  for (const auto filename : {
           "16MM2019H_Tashiro.maze",
           "16MM2019H_student.maze",
           "16MM2019H_Kyushu.maze",
           "16MM2019H_kansai.maze",
           "16MM2019H_kanazawa.maze",
           "16MM2019H_hokuriku.maze",
           "16MM2019H_East.maze",
           "16MM2018H_semi.maze",
           "16MM2017HX_pre.maze",
           "16MM2017H_Tashiro.maze",
           "16MM2017H_Cheese.maze",
           "16MM2017CX_pre.maze",
           "16MM2017C_East.maze",
           "16MM2016C_Kyushu.maze",
           "08MM2016CF_pre.maze",
       })
    filenames.push_back(filename);
#if 0
  for (const auto filename : {
           "04_test.maze",
           "32_fake.maze",
           "32_no_wall.maze",
           "32_unknown.maze",
       })
    filenames.push_back(filename);
#endif
#endif
  /* analyze for each maze */
  for (const auto &filename : filenames) {
    std::cout << std::endl;
    std::cout << "Maze File: \t" << filename << std::endl;
    csv << filename;

    /* Maze Target */
    const auto p_maze_target = std::make_unique<Maze>();
    Maze &maze_target = *p_maze_target;
    if (!maze_target.parse(mazedata_dir + filename)) {
      loge << "File Parse Error!" << std::endl;
      continue;
    }

#if 1
    /* Search Run */
    const auto p_robot = std::make_unique<CLRobot>(maze_target);
    CLRobot &robot = *p_robot;
    robot.replaceGoals(maze_target.getGoals());
    const auto t_s = std::chrono::system_clock().now();
    if (!robot.searchRun())
      loge << "Failed to Find a Path to Goal!" << std::endl;
    const auto t_e = std::chrono::system_clock().now();
    const auto t_search =
        std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s)
            .count();
    robot.printResult();
    csv << "," << robot.cost << "," << robot.step << "," << robot.f << ","
        << robot.l << "," << robot.r << "," << robot.b;
    csv << "," << robot.getMaze().getWallLogs().size();
    std::cout << "Max Calc Time:\t" << robot.t_dur_max << "\t[us]" << std::endl;
    csv << "," << robot.t_dur_max;
    std::cout << "Total Search:\t" << t_search << "\t[us]" << std::endl;
    csv << "," << t_search;
    // robot.getMaze().print();
    for (const auto diag_enabled : {false, true}) {
      if (!robot.calcShortestDirections(diag_enabled)) {
        loge << "Failed to Find a Shortest Path! "
             << (diag_enabled ? "diag" : "no_diag") << std::endl;
        continue;
      }
      const auto path_cost = robot.getSearchAlgorithm().getShortestCost();
      std::cout << "PathCost " << (diag_enabled ? "diag" : "no_d") << ":\t"
                << path_cost << "\t[ms]" << std::endl;
      csv << "," << path_cost;
      robot.fastRun(diag_enabled);
      // robot.printPath();
      robot.endFastRunBackingToStartRun();
      /* Shortest Path Comparison */
      const auto p_at = std::make_unique<Agent>(maze_target);
      Agent &at = *p_at;
      at.calcShortestDirections(diag_enabled);
      if (at.getShortestDirections() != robot.getShortestDirections()) {
        logw << "searched path is not shortest! "
             << (diag_enabled ? "(diag)" : "(no_diag)") << std::endl;
        logw << "target: " << at.getSearchAlgorithm().getShortestCost()
             << " search: " << robot.getSearchAlgorithm().getShortestCost()
             << std::endl;
        // at.printPath(); robot.printPath();
      }
    }
#endif

#if 1
    /* Position Identification Run */
    robot.t_dur_max = 0;
    float id_cost_max = 0;   /*< 探索時間 [秒] */
    float id_cost_min = 1e6; /*< 探索時間 [秒] */
    const auto p_step_map = std::make_unique<StepMap>();
    StepMap &step_map = *p_step_map;
    const auto p_maze_pi = std::make_unique<Maze>();
    Maze &maze_pi = *p_maze_pi;
    maze_pi = robot.getMaze(); /*< 探索終了時の迷路を取得 */
    /* 迷路的に行き得る区画を洗い出す */
    step_map.update(maze_target, {maze_target.getStart()}, true, true);
    for (int8_t x = 0; x < MAZE_SIZE; ++x)
      for (int8_t y = 0; y < MAZE_SIZE; ++y)
        for (const auto d : Direction::getAlong4()) {
          const auto p = Position(x, y);
          if (step_map.getStep(p) == StepMap::STEP_MAX)
            continue; /*< そもそも迷路的に行き得ない区画は除外 */
          if (maze_target.isWall(p, d + Direction::Back))
            continue; /*< 壁上からは除外 */
          if (p == Position(0, 0))
            continue; /*< スタートは除外 */
          /* set fake offset */
          robot.fake_offset = robot.real = Pose(Position(x, y), d);
          robot.setMaze(maze_pi); /*< 探索直後の迷路に置き換える */
          robot.setForceGoingToGoal(); /*< ゴールへの訪問を指定 */
          bool res = robot.positionIdentifyRun();
          if (!res) {
            loge << "Failed to Identify! fake_offset: " << robot.fake_offset
                 << std::endl;
          }
          /* save result */
          id_cost_max = std::max(id_cost_max, robot.cost);
          id_cost_min = std::min(id_cost_min, robot.cost);
        }
    std::cout << "P.I. Max Calc:\t" << robot.t_dur_max << "\t[us]" << std::endl;
    std::cout << "P.I. Time:\t" << (int(id_cost_min) / 60) % 60 << ":"
              << std::setw(2) << std::setfill('0') << int(id_cost_min) % 60
              << "\t" << (int(id_cost_max) / 60) % 60 << ":" << std::setw(2)
              << std::setfill('0') << int(id_cost_max) % 60 << std::setfill(' ')
              << std::endl;
    std::cout << "P.I. wall:\t" << robot.min_id_wall << "\t"
              << robot.max_id_wall << std::endl;
    csv << "," << robot.t_dur_max;
    csv << "," << id_cost_min;
    csv << "," << id_cost_max;
    csv << "," << robot.min_id_wall;
    csv << "," << robot.max_id_wall;
#endif

#if 0
    /* Shortest Algorithm */
    for (const auto diag_enabled : {false, true}) {
      const int n = 100;
      const bool known_only = 0;
      const Maze &maze = maze_target;
      const auto p_sa = std::make_unique<ShortestAlgorithm>(maze);
      ShortestAlgorithm &sa = *p_sa;
      Indexes path;
      std::chrono::microseconds sum{0};
      for (int i = 0; i < n; ++i) {
        const auto t_s = std::chrono::system_clock().now();
        if (!sa.calcShortestPath(path, known_only, diag_enabled))
          loge << "Failed!" << std::endl;
        const auto t_e = std::chrono::system_clock().now();
        const auto us =
            std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        sum += us;
      }
      std::cout << "Shortest " << (diag_enabled ? "diag" : "no_d") << ":\t"
                << sum.count() / n << "\t[us]" << std::endl;
      // sa.print(path);
    }
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
          loge << "Failed!" << std::endl;
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
          loge << "Failed!" << std::endl;
        const auto t_e = std::chrono::system_clock().now();
        const auto us =
            std::chrono::duration_cast<std::chrono::microseconds>(t_e - t_s);
        sum += us;
      }
      std::cout << "StepMapWall " << (simple ? "s" : "n") << ":\t"
                << sum.count() / n << "\t[us]" << std::endl;
      // map.print(maze, shortest_dirs);
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

    csv << std::endl;
  }
  std::cout << std::endl << "Measurement End" << std::endl;

  return 0;
}

int main(void) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  return test_measurement();
}
