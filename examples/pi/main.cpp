#include "MazeLib/CLRobotBase.h"

#include <thread>

using namespace MazeLib;

class CLRobot : public CLRobotBase {
 public:
  CLRobot(Maze& maze_target) : CLRobotBase(maze_target) {}
  bool display = 0;
  void printInfo() {
    CLRobotBase::printInfoDoubleMaze();
    std::cout << "P.I. walls:\t"
              << getSearchAlgorithm().getIdMaze().getWallRecords().size()
              << std::endl;
  }

 protected:
  virtual void calcNextDirectionsPostCallback(
      SearchAlgorithm::State oldState,
      SearchAlgorithm::State newState) override {
    CLRobotBase::calcNextDirectionsPostCallback(oldState, newState);
    if (newState == oldState)
      return;
    /* state change has occurred */
    if (oldState == SearchAlgorithm::IDENTIFYING_POSITION) {
      display = 0;
    }
  }
  virtual void crashed() override {
    printInfo();
    CLRobotBase::crashed();
    getc(stdin);
  }
  virtual void queueAction(const SearchAction action) override {
    if (display) {
      printInfo();
      // getc(stdin);
      // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
#if 1
    if (getState() == SearchAlgorithm::IDENTIFYING_POSITION &&
        real.p == maze.getStart() && action != ST_HALF_STOP)
      MAZE_LOGW << "Visited Start! fake_offset: " << fake_offset << std::endl;
#endif
    CLRobotBase::queueAction(action);
  }
};

int test_position_identify() {
  /* Preparation */
  const std::string mazedata_dir = "../mazedata/data/";
  const std::string filename = "32MM2021HX.maze";
  Maze maze_target;
  if (!maze_target.parse((mazedata_dir + filename).c_str()))
    return -1;
  const auto p_robot = std::make_unique<CLRobot>(maze_target);
  CLRobot& robot = *p_robot;
  robot.replaceGoals(maze_target.getGoals());
  robot.searchRun();
  // robot.printInfo();

  /* 探索終了時の迷路を保存 */
  const auto p_maze_pi = std::make_unique<Maze>();
  Maze& maze_pi = *p_maze_pi;
  maze_pi = robot.getMaze(); /*< 探索終了時の迷路を取得 */

#if 1
  /* Position Identification Run */
  robot.display = 1;
  robot.fake_offset = robot.real = Pose(Position(0, 5), Direction::North);
  robot.updateMaze(maze_pi); /*< 探索直後の迷路に置き換える */
  // robot.resetLastWalls(robot.getMaze().getWallRecords().size() / 2);
  robot.setForceGoingToGoal(); /*< ゴールへの訪問を指定 */
  bool res = robot.positionIdentifyRun();
  if (!res) {
    robot.printInfo();
    std::cout << std::endl
              << "Failed to Identify! offset:\t" << robot.fake_offset
              << std::endl;
    getc(stdin);
  }
#endif

#if 1
  /* Position Identification Run */
  const auto p_step_map = std::make_unique<StepMap>();
  StepMap& step_map = *p_step_map;
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
        /* set fake offset */
        robot.fake_offset = robot.real = Pose(Position(x, y), d);
        robot.updateMaze(maze_pi); /*< 探索直後の迷路に置き換える */
        // robot.resetLastWalls(robot.getMaze().getWallRecords().size() / 2);
        robot.setForceGoingToGoal(); /*< ゴールへの訪問を指定 */
        robot.display = 1;
        bool res = robot.positionIdentifyRun();
        if (!res) {
          robot.printInfo();
          std::cout << std::endl
                    << "Failed to Identify! fake_offset:\t" << robot.fake_offset
                    << std::endl;
          getc(stdin);
        }
      }
#endif
  std::cout << std::endl << "End" << std::endl;

  return 0;
}

int main(void) {
  return test_position_identify();
}
