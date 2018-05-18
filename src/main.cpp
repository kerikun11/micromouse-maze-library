#include <cstdio>
#include <cstdint>
#include "MazeLib/Maze.h"
#include "MazeLib/Agent.h"
#include "MazeLib/RobotBase.h"

#include <iostream>
#include <fstream>

#include "mazedata.h"

#include <unistd.h>
#include <time.h>
#include <chrono>

using namespace MazeLib;

#define DISPLAY		1

#if MAZE_SIZE == 8
// Vectors goal = {Vector(7,7)};
Vectors goal = {Vector(1,0)};
// Maze sample(mazeData_fp2016);
Maze sample(mazeData_a);
#elif MAZE_SIZE == 16
Vectors goal = {Vector(7,7),Vector(7,8),Vector(8,8),Vector(8,7)};
// Vectors goal = {Vector(7,7)};
// Vectors goal = {Vector(3,3),Vector(3,4),Vector(4,3),Vector(4,4)};
// Maze sample(mazeData_maze, false);
// Maze sample(mazeData_maze3, false);
// Maze sample(mazeData_maze4, false);
//Maze sample(mazeData_maze2013fr, false);
// Maze sample(mazeData_maze2013exp, false);
Maze sample(mazeData_2017_East_MC, true);
// Maze sample(mazeData_MM2017CXpre, true);
// Maze sample(mazeData_MM2017CX, true);
// Maze sample(mazeData_fp2016C);
// Maze sample(mazeData_Cheese2017, true);
#elif MAZE_SIZE == 32
#define YEAR 2017
#if YEAR == 2011
Vectors goal = {Vector(1,0)};
Maze sample(mazeData_32);
#elif YEAR == 2012
Vectors goal = {Vector(22,25)};
Maze sample(mazeData_MM2012HX);
#elif YEAR == 2013
Vectors goal = {Vector(6,5)};
Maze sample(mazeData_MM2013HX, false);
#elif YEAR == 2014
Vectors goal = {Vector(26,5)};
Maze sample(mazeData_MM2014HX);
#elif YEAR == 2015
Vectors goal = {Vector(7,24)};
Maze sample(mazeData_MM2015HX);
#elif YEAR == 2016
Vectors goal = {Vector(1,2), Vector(1,3), Vector(1,4), Vector(2,2), Vector(2,3), Vector(2,4), Vector(3,2), Vector(3,3), Vector(3,4)};
// Vectors goal = {Vector(1,2)};
Maze sample(mazeData_MM2016HX);
#elif YEAR == 2017
Vectors goal = {Vector(19,20), Vector(19,21), Vector(19,22), Vector(20,20), Vector(20,21), Vector(20,22), Vector(21,20), Vector(21,21), Vector(21,22)};
// Vectors goal = {Vector(19,20)};
Maze sample(mazeData_MM2017HX);
#endif
#endif

bool display = 0;
Vector offset(-4, -4);

class TestRobot : public RobotBase {
public:
	TestRobot(const Vectors& goal) : RobotBase(goal) { }

	void printInfo(const bool showMaze = true){
		Agent::printInfo(showMaze);
		printf("Cost: %5d, Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d, Known: %3d\n", cost, step, f, l, r, b, k);
		printf("It took %5d [us], the max is %5d [us]\n", (int)usec, (int)max_usec);
		usleep(50000);
		// char c; scanf("%c", &c);
	}
private:
	int step=0,f=0,l=0,r=0,b=0,k=0; /**< 探索の評価のためのカウンタ */
	int cost = 0;
	int max_usec = 0;
	int usec;
	std::chrono::_V2::system_clock::time_point start;
	std::chrono::_V2::system_clock::time_point end;

	void findWall(bool& left, bool& front, bool& right, bool& back) override {
		const auto& v = getCurVec();
		const auto& d = getCurDir();
		if (getState() == Agent::IDENTIFYING_POSITION) {
			// offset = Vector(13, 13);
			left  = sample.isWall(v+offset, d+Dir::Left);
			front = sample.isWall(v+offset, d+Dir::Front);
			right = sample.isWall(v+offset, d+Dir::Right);
			back  = sample.isWall(v+offset, d+Dir::Back);
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
	void calcNextDirsPostCallback(Agent::State prevState, Agent::State newState) override {
		end = std::chrono::system_clock::now();
		usec = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
		if(max_usec < usec) max_usec = usec;

		if(newState != prevState && newState == Agent::SEARCHING_ADDITIONALLY){ }
		if(newState != prevState && newState == Agent::BACKING_TO_START){ }
	}
	void queueAction(const Action action) override {
		#if DISPLAY
		if(display) {
			printInfo();
		}
		#endif
		cost += getCost(action);
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
	int getCost(const Action action) {
		switch (action) {
			case RobotBase::START_STEP:				return 5;
			case RobotBase::START_INIT:				return 10;
			case RobotBase::STOP_HALF:				return 10;
			case RobotBase::TURN_LEFT_90:			return 5;
			case RobotBase::TURN_RIGHT_90:		return 5;
			case RobotBase::ROTATE_LEFT_90:		return 5;
			case RobotBase::ROTATE_RIGHT_90:	return 5;
			case RobotBase::ROTATE_180:				return 10;
			case RobotBase::STRAIGHT_FULL:		return 3;
			case RobotBase::STRAIGHT_HALF:		return 2;
		}
		return 0;
	}
};

TestRobot robot(goal);

int main(void){
	setvbuf(stdout, (char *)NULL, _IONBF, 0);
	#if 1
	display = true;
	robot.searchRun();
	robot.printInfo();
	// while(!robot.searchRun());
	// robot.forceGoingToGoal();
	// while(!robot.positionIdentifyRun(Dir::West));
	// robot.positionIdentifyRun(Dir::West);
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

	#endif
	return 0;
}
