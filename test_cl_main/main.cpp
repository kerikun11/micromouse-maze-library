#include "Maze.h"
#include "Agent.h"
#include "RobotBase.h"
#include <cstdio>
#include <cstdint>

#include "mazedata.h"

#include <unistd.h>
#include <time.h>
#include <chrono>

using namespace MazeLib;

#if MAZE_SIZE == 8
Maze sample("../mazedata/08Test1.maze");
// Maze sample("../mazedata/08MM2016CF_pre.maze");
#elif MAZE_SIZE == 16
Maze sample("../mazedata/16MM2017CX.maze");
#elif MAZE_SIZE == 32
#define YEAR 2016
#if YEAR == 2012
Maze sample("../mazedata/32MM2012HX.maze");
#elif YEAR == 2013
Maze sample("../mazedata/32MM2013HX.maze");
#elif YEAR == 2014
Maze sample("../mazedata/32MM2014HX.maze");
#elif YEAR == 2015
Maze sample("../mazedata/32MM2015HX.maze");
#elif YEAR == 2016
Maze sample("../mazedata/32MM2016HX.maze");
#elif YEAR == 2017
Maze sample("../mazedata/32MM2017HX.maze");
#endif
#endif

bool display = 0;
Vector offset;

#if 1

class TestRobot : public RobotBase {
public:
	TestRobot(const Vectors& goal) : RobotBase(goal) { }

	void printInfo(const bool showMaze = true){
		Agent::printInfo(showMaze);
		printf("Estimated Time: %2d:%02d, Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d\n", ((int)cost/60)%60, ((int)cost)%60, step, f, l, r, b);
		printf("It took %5d [us], the max is %5d [us]\n", (int)usec, (int)max_usec);
		usleep(50000);
		// char c; scanf("%c", &c);
	}
private:
	int step=0,f=0,l=0,r=0,b=0; /**< 探索の評価のためのカウンタ */
	float cost = 0;
	int max_usec = 0;
	int usec;
	std::chrono::_V2::system_clock::time_point start;
	std::chrono::_V2::system_clock::time_point end;

	void findWall(bool& left, bool& front, bool& right, bool& back) override {
		const auto& v = getCurVec();
		const auto& d = getCurDir();
		if (getState() == SearchAlgorithm::IDENTIFYING_POSITION) {
			// offset = Vector(12, 12);
			left  = sample.isWall(v+offset, d+Dir::Left);
			front = sample.isWall(v+offset, d+Dir::Front);
			right = sample.isWall(v+offset, d+Dir::Right);
			back  = sample.isWall(v+offset, d+Dir::Back);
			return;
		} else {
			offset = v - Vector(MAZE_SIZE/2, MAZE_SIZE/2);
		}
		left  = sample.isWall(v, d+Dir::Left);
		front = sample.isWall(v, d+Dir::Front);
		right = sample.isWall(v, d+Dir::Right);
		back  = sample.isWall(v, d+Dir::Back);
	}
	void calcNextDirsPreCallback() override {
		start = std::chrono::system_clock::now();
	}
	void calcNextDirsPostCallback(SearchAlgorithm::State prevState, SearchAlgorithm::State newState) override {
		end = std::chrono::system_clock::now();
		usec = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
		if(max_usec < usec) max_usec = usec;

		// if(newState != prevState && newState == SearchAlgorithm::SEARCHING_ADDITIONALLY){ }
		// if(newState != prevState && newState == SearchAlgorithm::BACKING_TO_START){ }
	}
	void queueAction(const Action action) override {
		if(display) printInfo();
		cost += getTimeCost(action);
		step++;
		switch (action) {
			case RobotBase::START_STEP:  f++;
			break;
			case RobotBase::START_INIT:
			break;
			case RobotBase::STOP_HALF:
			break;
			case RobotBase::TURN_LEFT_90: l++;
			break;
			case RobotBase::TURN_RIGHT_90: r++;
			break;
			case RobotBase::ROTATE_LEFT_90:
			break;
			case RobotBase::ROTATE_RIGHT_90:
			break;
			case RobotBase::ROTATE_180: b++;
			break;
			case RobotBase::STRAIGHT_FULL: f++;
			break;
			case RobotBase::STRAIGHT_HALF:
			break;
		}
	}
	float getTimeCost(const Action action) {
		const float velocity = 240.0f;
		const float segment = 90.0f;
		switch (action) {
			case RobotBase::START_STEP:				return 1.0f;
			case RobotBase::START_INIT:				return 1.0f;
			case RobotBase::STOP_HALF:				return segment/2/velocity;
			case RobotBase::TURN_LEFT_90:			return 71/velocity;
			case RobotBase::TURN_RIGHT_90:		return 71/velocity;
			case RobotBase::ROTATE_LEFT_90:		return 0.5f;
			case RobotBase::ROTATE_RIGHT_90:	return 0.5f;
			case RobotBase::ROTATE_180:				return 2.0f;
			case RobotBase::STRAIGHT_FULL:		return segment/velocity;
			case RobotBase::STRAIGHT_HALF:		return segment/2/velocity;
		}
		return 0;
	}
};

TestRobot robot(sample.getGoals());

#endif

int main(void){
	setvbuf(stdout, (char *)NULL, _IONBF, 0);
	#if 1
	robot.replaceGoals(sample.getGoals());
	robot.searchRun();
	robot.printInfo();
	display = true;
	robot.forceGoingToGoal();
	robot.positionIdentifyRun(Dir::West);
	// for(int x=-MAZE_SIZE/2; x<MAZE_SIZE/2; ++x)
	// for(int y=-MAZE_SIZE/2; y<MAZE_SIZE/2; ++y){
	// 	offset = Vector(x, y);
	// 	bool res = robot.positionIdentifyRun(Dir::West);
	// 	if(!res) {
	// 		robot.printInfo();
	// 		display = true;
	// 		robot.positionIdentifyRun(Dir::West);
	// 		printf("Failed to Identify! (%3d, %3d)\n", x, y);
	// 		usleep(1000000);
	// 		display = false;
	// 	}
	// }
	robot.fastRun(true);
	// robot.endFastRunBackingToStartRun();
	robot.printPath();
	robot.fastRun(false);
	// robot.endFastRunBackingToStartRun();
	robot.printPath();
	#else
	// std::ifstream ifs("MM2016HX_ip.maze");
	// Maze m(ifs);
	// Maze m("MM2016HX_ip.maze");
	// m.parse(ifs);
	// m.print(std::cout);

	// sample.setGoals(goal);
	// std::ofstream ofs(filename);
	// sample.print(ofs);
	// sample.print();
	#endif
	return 0;
}
