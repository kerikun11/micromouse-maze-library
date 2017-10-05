#pragma once

#include "Maze.h"
#include "StepMap.h"

class Agent{
	public:
		Agent(Maze& maze, const std::vector<Vector>& goal) : maze(maze), stepMap(maze), goal(goal){ reset(); }
		enum State{
			IDOLE,
			SEARCHING_FOR_GOAL,
			REACHED_GOAL,
			SEARCHING_ADDITIONALLY,
			BACKING_TO_START,
			REACHED_START,
			FORCE_BACKING_TO_START,
			GOT_LOST,
		};
		static const char* stateString(const enum State s){
			static const char* str[]={
				"idole",
				"Searching for Goal",
				"Reached Goal",
				"Searching Additionally",
				"Backing to Start",
				"Reached Start",
				"Force Backing to Start",
				"Got Lost",
			};
			return str[s];
		}
		void reset(const std::vector<Vector>& goal) {
			this->goal = goal;
			reset();
		}
		void reset(){
			stepMap.reset();
			curVec = Vector(0, 0);
			curDir = Dir::North;
			state = IDOLE;
			calcNextDir();
		}
		void forceBackToStart(){
			if(state != REACHED_START) state = FORCE_BACKING_TO_START;
		}
		void updateCurVecDir(const Vector& v, const Dir& d){ curVec = v; curDir=d; }
		void updateWall(const Vector& v, const Dir& d, const bool& b){ maze.updateWall(v, d, b); }

		bool calcNextDir(){
			state = calcNextDir(curVec, curDir, state);
			return state != GOT_LOST;
		}
		bool calcShortestDirs(){
			stepMap.update(goal, StepMap::Goal, true);
			shortestDirs.clear();
			Vector v = start;
			Dir dir = Dir::North;
			Dir prev_dir = Dir::North;
			while(1){
				std::vector<Dir> dirs;
				if(Dir(dir-prev_dir)==Dir::Left) dirs={Dir(dir+3), dir, Dir(dir+1)};
				else if(Dir(dir-prev_dir)==Dir::Right) dirs={Dir(dir+1), dir, Dir(dir+3)};
				else dirs={dir, Dir(dir+1), Dir(dir+3)};
				auto it = std::find_if(dirs.begin(), dirs.end(),[&](const Dir& d){
						if(!maze.canGo(v, d)) return false;
						return stepMap.getStep(v.next(d)) == stepMap.getStep(v)-1;
						});
				if(it == dirs.end()) return false;
				prev_dir = dir;
				dir = *it;
				v=v.next(dir);
				shortestDirs.push_back(dir);
				if(stepMap.getStep(v)==0) break;
			}
			while(maze.canGo(v, dir)){
				shortestDirs.push_back(dir);
				v=v.next(dir);
			}
			return true;
		}
		const std::vector<std::vector<Dir>> calcNextDirInAdvance(){
			calcNextDir();
			std::vector<std::vector<Dir>> nextDirss;
			while(1){
				auto dirs = curDir.ordered();
				auto it = std::find_if(dirs.begin(), dirs.end(),[&](const Dir& d){
						if(maze.isWall(curVec, d)) return false;
						return stepMap.getStep(curVec.next(d)) == stepMap.getStep(curVec)-1;
						});
				if(it == dirs.end()) break;
				printf("curVec:(%d,%d), curDir:%d, *it:%d\n", curVec.x, curVec.y, int8_t(curDir), int8_t(*it));
				if(maze.isKnown(curVec, *it)){
					printf("isKnown; break;\n");
					calcNextDir();
					nextDirss.push_back(getNextDirs());
					break;
				}
				maze.setKnown(curVec, *it, true);
				if(calcNextDir(curVec, curDir, state)==GOT_LOST){
					//printInfo();
					break;
				}
				maze.setKnown(curVec, *it, false);
				nextDirss.push_back(getNextDirs());
				maze.setWall(curVec, *it, true);
			}
			for(auto d: Dir::All()) if(!maze.isKnown(curVec, d)) maze.setWall(curVec, d, false);
			calcNextDir();
			for(auto nd: nextDirss) {
				printf(">");
				for(auto d: nd) printf("%d ", int8_t(d));
				printf("\n");
			}
			return nextDirss;
		}

		const State& getState() const {
			return state;
		}
		const Maze& getMaze() const {
			return maze;
		}
		const std::vector<Dir>& getNextDirs() const {
			return nextDirs;
		}
		const Vector& getCurVec() const {
			return curVec;
		}
		const Dir& getCurDir() const {
			return curDir;
		}
		const std::vector<Dir>& getShortestDirs() const {
			return shortestDirs;
		}
		void printInfo(const bool& showMaze = true) const {
			if(showMaze){
				for(int i=0; i<MAZE_SIZE*2+4; i++) printf("\x1b[A");
				switch(state){
					case IDOLE:
					case SEARCHING_FOR_GOAL:
						stepMap.print(curVec, StepMap::Goal);
						break;
					case REACHED_GOAL:
					case SEARCHING_ADDITIONALLY:
						stepMap.print(curVec, StepMap::General);
						break;
					case BACKING_TO_START:
						stepMap.print(curVec, StepMap::Start);
						break;
					case REACHED_START:
					case GOT_LOST:
					default:
						stepMap.print(curVec, StepMap::Goal);
						break;
				}
			}
			printf("Cur: ( %3d, %3d, %3d), State: %s       \n", curVec.x, curVec.y, uint8_t(curDir), stateString(state));
			printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d\n", step, f, l, r, b);
		}
		void printPath() const {
			//for(int i=0; i<MAZE_SIZE*2+5; i++) printf("\x1b[A");
			maze.printPath(Vector(0, 0), shortestDirs);
			printf("\n\n");
			printf("Shortest Step: %d\n", shortestDirs.size()-1);
		}
	private:
		State state;
		Maze& maze;
		StepMap stepMap;
		const Vector start{0, 0};
		std::vector<Vector> goal;
		Vector curVec;
		Dir curDir;
		std::vector<Dir> nextDirs;
		std::vector<Dir> shortestDirs;
		std::vector<Vector> candidates;
		int step=0,f=0,l=0,r=0,b=0;

		bool calcNextDirByStepMap(const enum StepMap::Purpose& sp){
			nextDirs.clear();
			Vector focus_v = curVec;
			Dir focus_d = curDir;
			while(1){
				auto dirs = focus_d.ordered();
				auto it = std::find_if(dirs.begin(), dirs.end(), [&](const Dir& d){
						if(!maze.canGo(focus_v, d)) return false;
						return stepMap.getStep(focus_v.next(d), sp) == stepMap.getStep(focus_v, sp)-1;
						});
				if(it==dirs.end()) break;
				nextDirs.push_back(*it);
				focus_d = *it;
				focus_v = focus_v.next(*it);
			}
			if(nextDirs.empty()){
				return false;
			}
			return true;
		}
		void findShortestCandidates(){
			stepMap.update(goal, StepMap::Goal);
			stepMap.update({start}, StepMap::Start);
			candidates.clear();
			std::vector<step_t> goal_steps;
			for(auto g:goal) goal_steps.push_back(stepMap.getStep(g, StepMap::Start));
			step_t goal_step = *(std::min_element(goal_steps.begin(), goal_steps.end()));
			for(int i=0; i<MAZE_SIZE; i++){
				for(int j=0; j<MAZE_SIZE; j++){
					Vector v(i,j);
#if DEEPNESS == 0
					if(stepMap.getStep(i,j) + stepMap.getStep(i,j,StepMap::Start) <= goal_step && maze.knownCount(Vector(i,j))!=4){
						candidates.push_back(v);
					}
#elif DEEPNESS == 1
					if(stepMap.getStep(i,j) != MAZE_STEP_MAX && maze.nKnown(Vector(i,j))!=4){
						candidates.push_back(v);
					}
#endif
				}
			}
		}
		const enum State calcNextDir(const Vector& pv, const Dir& pd, enum State state){
			if(state == IDOLE){
				step=0; f=0; l=0; r=0; b=0;
				findShortestCandidates();
				if(candidates.empty()) state = BACKING_TO_START;
				else state = SEARCHING_FOR_GOAL;
#if SEARCHING_ADDITIALLY_AT_START
				state = SEARCHING_ADDITIONALLY;
#endif
			}

			if(state == SEARCHING_FOR_GOAL){
				if(std::find(goal.begin(), goal.end(), pv)!=goal.end()){
					state = REACHED_GOAL;
					candidates = goal;
				}else{
					stepMap.update(goal, StepMap::Goal);
					if(!calcNextDirByStepMap(StepMap::Goal)) return GOT_LOST;
				}
			}

			if(state == REACHED_GOAL){
				candidates.erase(std::find(candidates.begin(),candidates.end(), pv));
				if(candidates.empty()){
					state = SEARCHING_ADDITIONALLY;
				}else{
					stepMap.update(candidates, StepMap::General);
					if(!calcNextDirByStepMap(StepMap::General)) return GOT_LOST;
				}
			}

			if(state == SEARCHING_ADDITIONALLY){
				findShortestCandidates();
				if(candidates.empty()){
					state = BACKING_TO_START;
				}else{
					stepMap.update(candidates, StepMap::General);
					if(!calcNextDirByStepMap(StepMap::General)) return GOT_LOST;
				}
			}

			if(state == BACKING_TO_START){
				if(pv == start) {
					state = REACHED_START;
				}else{
					stepMap.update({start}, StepMap::Start);
					if(!calcNextDirByStepMap(StepMap::Start)) return GOT_LOST;
				}
			}

			if(state == FORCE_BACKING_TO_START){
				if(pv == start) {
					state = REACHED_START;
				}else{
					stepMap.update({start}, StepMap::Start, true);
					if(!calcNextDirByStepMap(StepMap::Start)) return GOT_LOST;
				}
			}

			for(auto d: nextDirs){
				step++;
				f += pd.getRelative(Dir::Forward) == d;
				l += pd.getRelative(Dir::Left   ) == d;
				r += pd.getRelative(Dir::Right  ) == d;
				b += pd.getRelative(Dir::Back   ) == d;
			}
			return state;
		}
};

