#pragma once

#include "Maze.h"

class StepMap{
	public:
		StepMap(Maze& maze) : maze(maze) { reset(); }
		enum Purpose : int8_t {
			Goal,
			Start,
			General,
			PurposeMax,
		};
		const StepMap& operator=(StepMap& obj){
			for(int8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					for(int sp=0; sp<PurposeMax; ++sp)
						getStep(x, y, static_cast<Purpose>(sp)) = obj.getStep(x, y, static_cast<Purpose>(sp));
			return *this;
		}
		void reset(){
			for(int8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					for(int sp=0; sp<PurposeMax; ++sp)
						getStep(x, y, static_cast<Purpose>(sp)) = 0;
		}
		inline step_t& getStep(const Vector& v, const enum Purpose& sp = Goal) { return getStep(v.x, v.y, sp); }
		inline step_t& getStep(const int8_t& x, const int8_t& y, const enum Purpose& sp = Goal) {
			static step_t outside;
			outside = MAZE_STEP_MAX;
			if(x<0 || y<0 || x>MAZE_SIZE-1 || y>MAZE_SIZE-1){
				printf("Warning: pefered out of field ------------------------------------------> %2d, %2d\n", x, y);
				return outside;
			}
			return stepMap[sp][y][x];
		}
		void print(const Vector& v=Vector(-1,-1), const enum Purpose& sp = Goal) const {
			maze.printWall(stepMap[sp], v);
		}
		void update(const std::vector<Vector>& dest, const enum Purpose& sp, const bool& onlyCanGo = false){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					getStep(x, y, sp) = MAZE_STEP_MAX;
			std::queue<Vector> q;
			for(auto v: dest) {
				getStep(v, sp) = 0;
				q.push(v);
			}
			while(!q.empty()){
				Vector focus = q.front(); q.pop();
				step_t focus_step = getStep(focus, sp);
				for(Dir d: Dir::All()){
					Vector next = focus.next(d);
					if(maze.isWall(focus, d)) continue;
					if(onlyCanGo && !maze.isKnown(focus, d)) continue;
					if(getStep(next, sp)>focus_step+1){
						getStep(next, sp) = focus_step+1;
						q.push(next);
					}
				}
			}
		}
	private:
		Maze& maze;
		step_t stepMap[PurposeMax][MAZE_SIZE][MAZE_SIZE];
};

