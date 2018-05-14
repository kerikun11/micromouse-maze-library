#include <cstdio>
#include <cstdint>
#include "MazeLib/Maze.h"
#include "MazeLib/Agent.h"

#include <iostream>

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
// Vectors goal = {Vector(7,7),Vector(7,8),Vector(8,8),Vector(8,7)};
Vectors goal = {Vector(7,7)};
// Vectors goal = {Vector(3,3),Vector(3,4),Vector(4,3),Vector(4,4)};
// Maze sample(mazeData_maze, false);
// Maze sample(mazeData_maze3, false);
// Maze sample(mazeData_maze4, false);
//Maze sample(mazeData_maze2013fr, false);
// Maze sample(mazeData_maze2013exp, false);
// Maze sample(mazeData_2017_East_MC, true);
Maze sample(mazeData_MM2017CXpre, true);
// Maze sample(mazeData_MM2017CX, true);
// Maze sample(mazeData_fp2016C);
// Maze sample(mazeData_Cheese2017, true);
#elif MAZE_SIZE == 32
#define YEAR 2017
#if YEAR == 2012
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

Agent agent(goal);
auto max_usec = 0;
auto start = std::chrono::system_clock::now();
auto end = std::chrono::system_clock::now();       // 計測終了時刻を保存
auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
int step=0,f=0,l=0,r=0,b=0,k=0; /**< 探索の評価のためのカウンタ */

#if 1

bool findWall(const Vector v, const Dir d) {
	if (agent.getState() == SearchAlgorithm::IDENTIFYING_POSITION) {
		Vector offset(14, -14);
		// Vector offset(-2, -2);
		return sample.isWall(v+offset, d);
	}
	return sample.isWall(v, d);
}

void stopAndSaveMaze(){
	/* queue Action::STOP */
	/* wait for queue being empty */
	/* stop the robot */
	/* backup maze to flash memory */
	// const auto& v = agent.getCurVec();
	// const auto& d = agent.getCurDir();
	// agent.updateCurVecDir(v.next(d + 2), d + 2); // u-turn
	/* queue Action::RETURN */
	/* queue Action::GO_HALF */
	/* start the robot */
}

bool display = 0;

void queueActions(const Dirs& nextDirs){
		if(agent.getState() == SearchAlgorithm::IDENTIFYING_POSITION) {
			display = true;
		}
	#if DISPLAY
	// usleep(200000);
	#endif
	for(const auto& nextDir: nextDirs){
		const auto& nextVec = agent.getCurVec().next(nextDir);
		#if DISPLAY
		if(display){
			agent.printInfo();
			printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d, Known: %3d\n", step, f, l, r, b, k);
			printf("It took %5d [us], the max is %5d [us]\n", (int)usec, (int)max_usec);
			usleep(100000);
			char c; scanf("%c", &c);
		}
		#endif
		switch (Dir(nextDir - agent.getCurDir())) {
			case Dir::Forward:
			/* queue SearchRun::GO_STRAIGHT */
			f++;
			break;
			case Dir::Left:
			/* queue SearchRun::TURN_LEFT_90 */
			l++;
			break;
			case Dir::Right:
			/* queeu SearchRun::TURN_RIGHT_90 */
			r++;
			break;
			case Dir::Back:
			/* queue SearchRun::TURN_BACK */
			b++;
			break;
		}
		agent.updateCurVecDir(nextVec, nextDir);
		step++;
	}
}


bool searchRun(const bool isStartStep = true, const Vector& startVec = Vector(0, 0), const Dir& startDir = Dir::North){
	if(isStartStep) {
		agent.updateCurVecDir(startVec, startDir);
		/* queue Action::START_STEP */
		agent.updateCurVecDir(startVec.next(startDir), startDir);
	}
	/* conduct calibration of sensors */
	/* start the robot */
	int cnt=0;
	while(1){
		// if(cnt++ > 200) return false;
		const auto& v = agent.getCurVec();
		const auto& d = agent.getCurDir();
		SearchAlgorithm::State prevState = agent.getState();
		start = std::chrono::system_clock::now();
		const auto result = agent.calcNextDirs(); //< 時間がかかる処理！
		end = std::chrono::system_clock::now();
		usec = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
		if(max_usec < usec) max_usec = usec;
		SearchAlgorithm::State newState = agent.getState();
		if(!result) {
			/* queue SearchRun::STOP */
			/* wait for queue being empty */
			/* stop the robot */
			agent.printInfo();
			printf("Got Lost!");
			while (1);
			return false;
		}
		// if(newState != prevState && newState == SearchAlgorithm::REACHED_GOAL){ }
		// if(newState != prevState && newState == SearchAlgorithm::SEARCHING_ADDITIONALLY){ }
		// if(newState != prevState && newState == SearchAlgorithm::BACKING_TO_START){ }

		// 既知区間移動をキューにつめる
		queueActions(agent.getNextDirs());
		k += agent.getNextDirs().size();

		// reached start and searching finised
		if(v == Vector(0, 0)) break;

		/* wait for queue being empty */

		// find walls
		Dir nextDirCandidate;
		bool res = agent.updateWall(v, d, findWall(v, d+1), findWall(v, d), findWall(v, d-1), findWall(v, d+2), nextDirCandidate);
		if(agent.getNextDirCandidates().empty()) {
			printf("nextDirCandidates is empty!\n");
			while(1);
		}

		/* backup the wall */

		queueActions({nextDirCandidate});
	}
	/* queue Action::START_INIT */
	agent.updateCurVecDir(Vector(0, 0), Dir::North);
	agent.calcNextDirs(); //< 時間がかかる処理！
	/* wait for queue being empty */
	/* stop the robot */
	/* backup the maze */
	return true;
}

bool fastRun(){
	if(!agent.calcShortestDirs()){
		printf("Failed to find shortest path!\n");
		return false;
	}
	/* move robot here */
	return true;
}

#endif

int main(void){
	setvbuf(stdout, (char *)NULL, _IONBF, 0);
	#if 1
	searchRun();
	// agent.getMaze() = sample;
	agent.positionIdentify(Dir::East);
	searchRun(false);
	agent.printInfo();
	printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d, Known: %3d\n", step, f, l, r, b, k);
	printf("the max is %5d [us]\n", max_usec);
	fastRun();
	agent.printPath();
	agent.calcShortestDirs(false);
	agent.printPath();
	#else

	// StepMap stepMap(maze);
	// stepMap.update({Vector(0,31)});
	// stepMap.print();
	#endif
	return 0;
}
