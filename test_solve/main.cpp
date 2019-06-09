#include "CLRobotBase.h"

using namespace MazeLib;

class CLRobot : public CLRobotBase {
public:
  CLRobot(const Maze &maze_target) : CLRobotBase(maze_target) {}

protected:
  virtual void discrepancyWithKnownWall() override {
    printInfo();
    std::cout << "There was a discrepancy with known information! CurVecDir:\t"
              << VecDir{getCurVec(), getCurDir()} << std::endl;
    getc(stdin);
  }
  virtual void crashed() override {
    printInfo();
    CLRobotBase::crashed();
    getc(stdin);
  }
};

int main(int argc, char *argv[]) {
  setvbuf(stdout, (char *)NULL, _IONBF, 0);
  if (argc < 2) {
    std::cout << "Please specify a maze file!" << std::endl;
    std::cout << "usage: $ test_solve <mazefile.maze>" << std::endl;
    return -1;
  }
  const auto filename = argv[1];
  Maze maze_target;
  if (!maze_target.parse(filename)) {
    std::cout << "Failed to parse " << filename << " !" << std::endl;
    return -1;
  }
  std::cout << "Solving " << filename << " ..." << std::endl;
  CLRobot robot(maze_target);
  robot.replaceGoals(maze_target.getGoals());
  robot.searchRun();
  robot.printInfo();
  robot.fastRun(false);
  robot.endFastRunBackingToStartRun();
  robot.printPath();
  robot.fastRun(true);
  robot.endFastRunBackingToStartRun();
  robot.printPath();
  return 0;
}
