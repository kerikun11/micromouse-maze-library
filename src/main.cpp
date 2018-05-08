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

#define DISPLAY						1

#if MAZE_SIZE == 8
// std::vector<Vector> goal = {Vector(7,7)};
std::vector<Vector> goal = {Vector(1,0)};
// Maze sample(mazeData_fp2016);
Maze sample(mazeData_a);
#elif MAZE_SIZE == 16
std::vector<Vector> goal = {Vector(7,7),Vector(7,8),Vector(8,8),Vector(8,7)};
// std::vector<Vector> goal = {Vector(3,3),Vector(3,4),Vector(4,3),Vector(4,4)};
// Maze sample(mazeData_maze, false);
// Maze sample(mazeData_maze3, false);
// Maze sample(mazeData_maze4, false);
//Maze sample(mazeData_maze2013fr, false);
// Maze sample(mazeData_maze2013exp, false);
// Maze sample(mazeData_2017_East_MC, true);
Maze sample(mazeData_MM2017CXpre, true);
//Maze sample(mazeData_MM2017CX, true);
// Maze sample(mazeData_Cheese2017, true);
#elif MAZE_SIZE == 32
#define YEAR 2015
#if YEAR == 2012
std::vector<Vector> goal = {Vector(22,25)};
Maze sample(mazeData_MM2012HX);
#elif YEAR == 2013
std::vector<Vector> goal = {Vector(6,5)};
Maze sample(mazeData_MM2013HX, false);
#elif YEAR == 2014
std::vector<Vector> goal = {Vector(26,5)};
Maze sample(mazeData_MM2014HX);
#elif YEAR == 2015
std::vector<Vector> goal = {Vector(7,24)};
Maze sample(mazeData_MM2015HX);
#elif YEAR == 2016
std::vector<Vector> goal = {Vector(1,2), Vector(1,3), Vector(1,4), Vector(2,2), Vector(2,3), Vector(2,4), Vector(3,2), Vector(3,3), Vector(3,4)};
// std::vector<Vector> goal = {Vector(1,2)};
Maze sample(mazeData_MM2016HX);
#elif YEAR == 2017
std::vector<Vector> goal = {Vector(19,20), Vector(19,21), Vector(19,22), Vector(20,20), Vector(20,21), Vector(20,22), Vector(21,20), Vector(21,21), Vector(21,22)};
// std::vector<Vector> goal = {Vector(19,20)};
Maze sample(mazeData_MM2017HX);
#endif
#endif

Maze maze;
Agent agent(maze, goal);
auto max_usec = 0;
auto start = std::chrono::system_clock::now();
auto end = std::chrono::system_clock::now();       // 計測終了時刻を保存
auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
int step=0,f=0,l=0,r=0,b=0,k=0; /**< 探索の評価のためのカウンタ */
int wall_log=0,log_max=0;

#if 1

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

void queueActions(const std::vector<Dir>& nextDirs){
	#if DISPLAY
	// usleep(200000);
	#endif
	for(const auto& nextDir: nextDirs){
		const auto& nextVec = agent.getCurVec().next(nextDir);
		#if DISPLAY
		agent.printInfo();
		printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d, Known: %3d\n", step, f, l, r, b, k);
		printf("It took %5d [us], the max is %5d [us]\n", usec, max_usec);
		printf("wall_log: %5d, log_max: %5d\n", wall_log, log_max);
		usleep(100000);
		char c; scanf("%c", &c);
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
			wall_log=0;
			break;
		}
		agent.updateCurVecDir(nextVec, nextDir);
		step++;
	}
}

bool searchRun(const bool isStartStep = true, const Vector& startVec = Vector(0, 0), const Dir& startDir = Dir::North){
	agent.reset();
	agent.updateCurVecDir(startVec, startDir);
	agent.calcNextDirs();
	if(agent.getState() == SearchAlgorithm::REACHED_START) return true;
	if(isStartStep) {
		/* queue Action::START_STEP */
		agent.updateCurVecDir(startVec.next(startDir), startDir);
	}
	/* conduct calibration of sensors */
	/* start the robot */
	int count=0;
	// agent.forceBackToStart(); // for debug
	while(1){
		// if(count++>50) return false; // for debug
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
			printf("\n");
			printf("Got Lost!");
			while (1);
			return false;
		}
		if(newState != prevState && newState == SearchAlgorithm::REACHED_GOAL){ }
		if(newState != prevState && newState == SearchAlgorithm::SEARCHING_ADDITIONALLY){ }
		if(newState != prevState && newState == SearchAlgorithm::BACKING_TO_START){ }

		// 既知区間移動をキューにつめる
		queueActions(agent.getNextDirs());
		k += agent.getNextDirs().size();

		// reached start and searching finised
		if(v == Vector(0, 0)) break;

		/* wait for queue being empty */

		// find walls
		if(!maze.isKnown(v, d+1)) wall_log++;
		if(!maze.isKnown(v, d+0)) wall_log++;
		if(!maze.isKnown(v, d-1)) wall_log++;
		agent.updateWall(v, d+1, sample.isWall(v, d+1)); // left wall
		agent.updateWall(v, d+0, sample.isWall(v, d+0)); // front wall
		agent.updateWall(v, d-1, sample.isWall(v, d-1)); // right wall
		if(log_max < wall_log) log_max = wall_log;
		// if(!maze.isWall(v,d)) agent.updateWall(v.next(d), d, sample.isWall(v.next(d), d)); // front wall
		/* backup the wall */

		// 候補の中で行ける方向を探す
		const auto nextDirsInAdvance = agent.getNextDirsInAdvance();
		const auto nextDirInAdvance = *std::find_if(nextDirsInAdvance.begin(), nextDirsInAdvance.end(), [&](const Dir& dir){
			return maze.canGo(v, dir);
		});
		queueActions({nextDirInAdvance});
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

#else

PositionIdentifier pi;

void queueActions(const std::vector<Dir>& nextDirs){
	#if DISPLAY
	// usleep(200000);
	#endif
	for(const auto& nextDir: nextDirs){
		const auto& nextVec = pi.getCurVec().next(nextDir);
		#if DISPLAY
		pi.printInfo();
		printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d, Known: %3d\n", step, f, l, r, b, k);
		printf("It took %5d [us], the max is %5d [us]\n", usec, max_usec);
		printf("wall_log: %5d, log_max: %5d\n", wall_log, log_max);
		usleep(100000);
		char c; scanf("%c", &c);
		#endif
		switch (Dir(nextDir - pi.getCurDir())) {
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
			wall_log=0;
			break;
		}
		pi.updateCurVecDir(nextVec, nextDir);
		step++;
	}
}

#endif

int main(void){
	setvbuf(stdout, (char *)NULL, _IONBF, 0);
	#if 1
	while(!searchRun());
	agent.printInfo();
	printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d, Known: %3d\n", step, f, l, r, b, k);
	printf("the max is %5d [us]\n", max_usec);
	printf("the log_max is %5d\n", log_max);
	fastRun();
	agent.printPath();
	agent.calcShortestDirs(false);
	agent.printPath();
	#else
	// sample.print();
	// maze = sample;
	// std::vector<WallLog> tmpWall;
	// // tmpWall.push_back(WallLog(-1, -1, Dir::East, false));
	// // tmpWall.push_back(WallLog(-1, -1, Dir::North, false));
	// tmpWall.push_back(WallLog(0, -1, Dir::East, true));
	// tmpWall.push_back(WallLog(0, -1, Dir::North, false));
	// tmpWall.push_back(WallLog(-1, 0, Dir::East, true));
	// tmpWall.push_back(WallLog(-1, 0, Dir::North, false));
	// tmpWall.push_back(WallLog(0, 0, Dir::East, true));
	// tmpWall.push_back(WallLog(0, 0, Dir::North, false));
	// tmpWall.push_back(WallLog(-1, 1, Dir::East, true));
	// tmpWall.push_back(WallLog(-1, 1, Dir::North, true));
	// tmpWall.push_back(WallLog(0, 1, Dir::East, false));
	// tmpWall.push_back(WallLog(0, 1, Dir::North, true));
	// tmpWall.push_back(WallLog(1, 1, Dir::East, true));
	// tmpWall.push_back(WallLog(1, 1, Dir::North, true));
	// tmpWall.push_back(WallLog(1, 0, Dir::East, true));
	// tmpWall.push_back(WallLog(1, 0, Dir::North, false));
	// tmpWall.push_back(WallLog(1, -1, Dir::East, true));
	// tmpWall.push_back(WallLog(1, -1, Dir::North, false));
	// tmpWall.push_back(WallLog(1, -2, Dir::East, true));
	// tmpWall.push_back(WallLog(1, -2, Dir::North, false));
	// tmpWall.push_back(WallLog(0, -2, Dir::East, false));
	// tmpWall.push_back(WallLog(0, -2, Dir::North, true));
	// Maze tmpMaze;
	// for(auto wl: tmpWall) tmpMaze.updateWall(Vector(wl.x+MAZE_SIZE/2, wl.y+MAZE_SIZE/2), wl.d, wl.b);
	// tmpMaze.print();
	// int cnt = 0;
	// for(int x=-MAZE_SIZE; x<MAZE_SIZE; x++)
	// for(int y=-MAZE_SIZE; y<MAZE_SIZE; y++) {
	// 	int diffs=0;
	// 	for(auto wl: tmpWall){
	// 		Vector v(wl.x+x, wl.y+y);
	// 		Dir d = wl.d;
	// 		if(maze.isKnown(v, d) && maze.isWall(v, d) != wl.b) diffs++;
	// 	}
	// 	if(diffs == 0) {
	// 		cnt++;
	// 		printf("%3d, %3d: %3d\n", x, y, diffs);
	// 	}
	// }
	// printf("cnt: %d\n", cnt);
	// printf("%d, %d\n", ans.x, ans.y);
	// maze = sample;
	// for(int x=0; x<MAZE_SIZE; x++){
	// 	for(int y=0; y<MAZE_SIZE; y++) {
	// 		for(const Dir d: {Dir::East, Dir::North}){
	// 			Vector v = Vector(x, y);
	// 			pi.updateWall(v, d, maze.isWall(v,d));
	// 			Vector ans;
	// 			auto cnt = pi.identify(maze, ans);
	// 			std::cout << "v: " << v << " d: " << d << " cnt: " << cnt << " ans: " << ans << std::endl;
	// 			if(cnt == 1) goto idd;
	// 		}
	// 	}
	// }
	// idd:
	maze = sample;
	while(1){
		const auto& v = pi.getCurVec();
		const auto& d = pi.getCurDir();
		pi.calcNextDirs();
		// pi.printInfo();
		Vector ans;
		auto cnt = pi.identify(maze, ans);
		std::cout << "v: " << v << " d: " << d << " cnt: " << cnt << " ans: " << ans << std::endl;
		if(cnt == 1) break;
		if(cnt == 0) break;

		// 既知区間移動をキューにつめる
		queueActions(pi.getNextDirs());

		// find walls
		const Vector offset = Vector(12, 6) - Vector(MAZE_SIZE/2, MAZE_SIZE/2);
		pi.updateWall(v, d+1, sample.isWall(v+offset, d+1)); // left wall
		pi.updateWall(v, d+0, sample.isWall(v+offset, d+0)); // front wall
		pi.updateWall(v, d-1, sample.isWall(v+offset, d-1)); // right wall
		// pi.updateWall(v, d+2, false); // right wall
		/* backup the wall */

		// 候補の中で行ける方向を探す
		const auto nextDirsInAdvance = pi.getNextDirsInAdvance();
		const auto nextDirInAdvance = *std::find_if(nextDirsInAdvance.begin(), nextDirsInAdvance.end(), [&](const Dir& dir){
			return pi.getMaze().canGo(v, dir);
		});
		queueActions({nextDirInAdvance});
	}

	// StepMap stepMap(maze);
	// stepMap.update({Vector(0,31)});
	// stepMap.print();
	#endif
	return 0;
}
