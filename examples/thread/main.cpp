#include "MazeLib/CLRobotBase.h"

#include <thread>

using namespace MazeLib;

class CLRobot : public CLRobotBase {
 public:
  CLRobot(Maze& maze_target) : CLRobotBase(maze_target) {}
  bool positionIdentifyRun() {
    /* Position Identification Run */
    StepMap step_map;
    Maze maze_searched = getMaze(); /*< 探索終了時の迷路を取得 */
    /* 迷路的に行き得る区画を洗い出す */
    step_map.update(maze_target, {maze_target.getStart()}, true, true);
    for (int8_t x = 0; x < MAZE_SIZE; ++x) {
      for (int8_t y = 0; y < MAZE_SIZE; ++y) {
        for (const auto d : Direction::Along4) {
          const auto p = Position(x, y);
          if (p == Position(0, 0))
            continue; /*< スタート区画は除外 */
          if (step_map.getStep(p) == StepMap::STEP_MAX)
            continue; /*< そもそも迷路的に行き得ない区画は除外 */
          if (maze_target.isWall(p, d + Direction::Back))
            continue; /*< 壁上は除外 */
          fake_offset = real = Pose(p, d);
          updateMaze(maze_searched); /*< 探索終了時の迷路に置き換える */
          setForceGoingToGoal(); /*< ゴールへの訪問を指定 */
          const bool res = CLRobotBase::positionIdentifyRun();
          if (!res)
            maze_loge << "Failed to Identify! fake_offset: " << fake_offset
                      << std::endl;
        }
      }
    }
    updateMaze(maze_searched); /*< 探索終了時の迷路に置き換える */
    return true;
  }

 protected:
  virtual void queueAction(const SearchAction action) override {
#if 0
    if (getState() == SearchAlgorithm::IDENTIFYING_POSITION &&
        real.p == maze.getStart() && action != ST_HALF_STOP)
      maze_logw << "Visited Start! fake_offset: " << fake_offset << std::endl;
#endif
    CLRobotBase::queueAction(action);
  }
};

void thread_maze(const std::string& name) {
  /* Maze Target */
  const std::string& mazedata_dir = "../mazedata/data/";
  const auto filepath = mazedata_dir + name + ".maze";
  Maze maze_target;
  if (!maze_target.parse(filepath)) {
    maze_loge << "File Parse Error!" << std::endl;
    return;
  }

  /* Search Run */
  CLRobot robot(maze_target);
  robot.searchRun();
  /* Fast Run */
  for (const auto diag_enabled : {false, true})
    robot.fastRun(diag_enabled);
#if 1
  /* Position Identification Run */
  robot.positionIdentifyRun();
#endif
  /* Show Result */
  std::printf("%-20s", name.c_str());
  robot.printSearchResult();
}

int main(void) {
  /* prepare target maze names */
  std::vector<std::string> names;
  for (int year = 2021; year >= 2021; --year)
    names.push_back("32MM" + std::to_string(year) + "HX");
#if 1
  for (int year = 2019; year >= 2010; --year)
    names.push_back("32MM" + std::to_string(year) + "HX");
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

  /* analyze for each maze */
  std::vector<std::thread> threads;
  for (const auto& name : names) {
    threads.push_back(std::thread([&] { thread_maze(name); }));
  }
  for (std::thread& th : threads) {
    th.join();
  }

  return 0;
}
