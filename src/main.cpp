#include <cstdio>
#include <queue>
#include <vector>
#include <array>
#include <cstdint>
#include <algorithm>
#include <unistd.h>

#define MAZE_SIZE		16
#define MAZE_STEP_MAX	999

#define C_RED     "\x1b[31m"
#define C_GREEN   "\x1b[32m"
#define C_YELLOW  "\x1b[33m"
#define C_BLUE    "\x1b[34m"
#define C_MAGENTA "\x1b[35m"
#define C_CYAN    "\x1b[36m"
#define C_RESET   "\x1b[0m"

typedef int8_t dir_t;
typedef uint16_t step_t;

class Dir{
	public:
		enum AbsoluteDir: uint8_t { East, North, West, South, AbsMax };
		Dir(const enum AbsoluteDir d = East):d(d){}
		Dir(const uint8_t d):d(AbsoluteDir(d&3)){}

		operator uint8_t() const { return d; }
		const Dir operator=(const Dir& obj) { this->d = obj.d; return *this; }

		const std::array<Dir, 4> ordered() const {
			std::array<Dir, 4> order{Forward(), Left(), Right(), Back()};
			return order;
		}

		const Dir Forward() const { return Dir(d); }
		const Dir Left() const { return Dir(d+1); }
		const Dir Right() const { return Dir(d+3); }
		const Dir Back() const { return Dir(d+2); }

		static const std::array<Dir, 4>& All(){
			static const std::array<Dir, 4> all = {East, North, West, South};
			return all;
		}
	private:
		enum AbsoluteDir d;
};

union Wall{
	Wall(uint8_t value = 0) : flags(value) {}
	uint8_t flags;
	struct{
		uint8_t e:1;	// east
		uint8_t n:1;	// north
		uint8_t w:1;	// west
		uint8_t s:1;	// south
		uint8_t E:1;	// known east
		uint8_t N:1;	// known north
		uint8_t W:1;	// known west
		uint8_t S:1;	// known south
	};
	inline operator uint8_t() const { return flags; }
	inline uint8_t operator[](uint8_t index) const { return (flags>>index)&0x01; }
	inline uint8_t operator=(const Wall& obj) { flags=obj.flags; return flags; }
	inline uint8_t operator|=(const Wall& obj) { flags|=obj.flags; return flags; }
	inline uint8_t operator&=(const Wall& obj) { flags&=obj.flags; return flags; }

	inline void reset(){ flags=0; }
	//inline int8_t nWall() const { return e+n+w+s; }
	inline int8_t nWall() const { return __builtin_popcount(0x0f&flags); }
	//inline int8_t nDone() const { return E+N+W+S; }
	inline int8_t nDone() const { return __builtin_popcount(0xf0&flags);; }

	inline void updateOne(const Dir &dir, const bool b) {
		if(b) flags |=   1<<dir;
		else  flags &= ~(1<<dir);
		flags |= 1<<(dir+4);
	}
	inline void updateAll(const uint8_t wall) {
		flags = 0xF0 | (wall&0x0F);
	}
	inline uint8_t rotate(const Dir &dir) const {
		return ((flags<<dir)|(flags>>(4-dir)))&0x0F;
	}
};

struct Vector{
	Vector(int8_t x=0, int8_t y=0) : x(x), y(y) {}
	int8_t x,y;

	const Vector& operator=(const Vector& obj) { x=obj.x; y=obj.y; return *this; }
	const Vector& operator+=(const Vector& obj) { x+=obj.x; y+=obj.y; return *this; }
	const Vector& operator-=(const Vector& obj) { x-=obj.x; y-=obj.y; return *this; }
	const Vector operator+(const Vector& obj) const { return Vector(x+obj.x, y+obj.y); }
	const Vector operator-(const Vector& obj) const { return Vector(x-obj.x, y-obj.y); }
	const bool operator==(const Vector& obj) const { return x==obj.x && y==obj.y; }
	const bool operator!=(const Vector& obj) const { return x!=obj.x || y!=obj.y; }

	const Vector next(const Dir &dir) const {
		switch(dir){
			case Dir::East: return Vector(x+1, y);
			case Dir::North: return Vector(x, y+1);
			case Dir::West: return Vector(x-1, y);
			case Dir::South: return Vector(x, y-1);
		}
	}
};

class Maze{
	public:
		Maze(){
			reset();
		}
		Maze(const char data[MAZE_SIZE+1][MAZE_SIZE+1], bool east_origin = true){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					char c = data[MAZE_SIZE-y-1][x];
					uint8_t h;
					if ('0' <= c && c <= '9') {
						h = c-'0';
					} else if('a'<=c && c<='f'){
						h = c-'a'+10;
					}
					if(east_origin){
						getWall(x, y).updateAll(h);
					}else{
						getWall(x, y).updateOne(0, h&0x02);
						getWall(x, y).updateOne(1, h&0x01);
						getWall(x, y).updateOne(2, h&0x08);
						getWall(x, y).updateOne(3, h&0x04);
					}
				}
		}
		inline void reset(){
			//* clear all wall
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					getWall(x, y).reset();
			//* set outline
			for(uint8_t i=0; i<MAZE_SIZE; i++){
				getWall(i, 0).updateOne(Dir::South, true);
				getWall(0, i).updateOne(Dir::West, true);
				getWall(MAZE_SIZE-1, i).updateOne(Dir::East, true);
				getWall(i, MAZE_SIZE-1).updateOne(Dir::North, true);
			}
			//* start cell
			updateWall(0,0,0x0b);
		}
		inline Wall& getWall(const Vector& v) { return getWall(v.x, v.y); }
		inline Wall& getWall(const int x, const int y) {
			static Wall edge;
			edge.flags = 0xFF;
			if(x<0 || y<0 || x>MAZE_SIZE-1 || y>MAZE_SIZE-1) return edge;
			return wall[y][x];
		}
		inline step_t& getStep(const Vector& v) { return stepMap[v.y][v.x]; }
		inline step_t& getStep(const int x, const int y) {
			static step_t outside;
			outside = MAZE_STEP_MAX;
			if(x<0 || y<0 || x>MAZE_SIZE-1 || y>MAZE_SIZE-1){
				printf("Warning: pefered out of field\n");
				return outside;
			}
			return stepMap[y][x];
		}
		inline void updateWall(const int x, const int y, const Wall& w){ return updateWall(Vector(x,y),w); }
		inline void updateWall(const Vector& v, Wall w){
			if(v.x==0) w.updateOne(Dir::West, true);
			if(v.y==0) w.updateOne(Dir::South, true);
			if(v.x==MAZE_SIZE-1) w.updateOne(Dir::East, true);
			if(v.y==MAZE_SIZE-1) w.updateOne(Dir::North, true);
			getWall(v) = w;
			for(Dir d: Dir::All()) getWall(v.next(d)).updateOne(2+d, w[d]); //< also update every next wall
		}
		inline void printWall(step_t nums[MAZE_SIZE][MAZE_SIZE] = NULL, const Vector v = Vector(-1,-1)){
#if 0
			for(int8_t y=MAZE_SIZE-1; y>=0; y--){
				for(uint8_t x=0; x<MAZE_SIZE; x++) printf("+%s+", wall[y][x].n ? "---" : "   ");
				printf("\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s", wall[y][x].w ? "|" : " ");
					if(nums==NULL) printf("   ");
					else printf("%3d", nums[y][x]);
					printf("%s", wall[y][x].e ? "|" : " ");
				}
				printf("\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++) printf("+%s+", wall[y][x].s ? "---" : "   ");
				printf("\n");
			}
#else
			printf("\n");
			for(int8_t y=MAZE_SIZE-1; y>=0; y--){
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					printf("+%s" C_RESET, wall[y][x].N ? (wall[y][x].n?"---":"   ") : C_RED " - ");
				printf("+\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s" C_RESET, wall[y][x].W ? (wall[y][x].w?"|":" ") : C_RED ":");
					if(nums!=NULL) printf("%s%3d" C_RESET, v==Vector(x,y)?C_YELLOW:C_CYAN, nums[y][x]);
					else printf("%s" C_RESET, v==Vector(x,y)?(C_YELLOW " X "):"   ");
				}
				printf("%s" C_RESET, wall[y][MAZE_SIZE-1].E ? (wall[y][MAZE_SIZE-1].e?"|":" ") : C_RED ":");
				printf("\n");
			}
			for(uint8_t x=0; x<MAZE_SIZE; x++)
				printf("+%s" C_RESET, wall[0][x].S ? (wall[0][x].s?"---":"   ") : C_RED " - ");
			printf("+\n\n");
#endif
		}
		inline void printPath(std::vector<Vector> path){
			step_t steps[MAZE_SIZE][MAZE_SIZE]={0};
			for(auto &v: path){
				steps[v.y][v.x] = (&v-&path[0]+1);
			}
			printWall(steps);
		}
		inline void printStepMap(const Vector v=Vector(-1,-1)){
			printWall(stepMap, v);
		}
		inline void updateStepMap(std::vector<Vector> dest){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					stepMap[y][x] = MAZE_STEP_MAX;
			for(auto d: dest) getStep(d) = 0;
			std::queue<Vector> q;
			for(auto d: dest) q.push(d);
			while(!q.empty()){
				Vector focus = q.front(); q.pop();
				step_t focus_step = getStep(focus);
				Wall focus_wall = getWall(focus);
				for(Dir dir: Dir::All()){
					Vector next = focus.next(dir);
					if(!focus_wall[dir] && getStep(next)>focus_step+1){
						getStep(next) = focus_step+1;
						q.push(next);
					}
				}
			}
		}
	private:
		Wall wall[MAZE_SIZE][MAZE_SIZE];
		uint16_t stepMap[MAZE_SIZE][MAZE_SIZE];
};

class MazeAgent{
	public:
		MazeAgent(std::vector<Vector> goal):state(IDOLE), goal(goal){
		}

		enum State{
			IDOLE,
			SEARCHING_FOR_GOAL,
			REACHED_GOAL,
			SEARCHING_ADDITIONALLY,
			BACKING_TO_START,
			REACHED_START,
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
				"Got Lost",
			};
			return str[s];
		}

		void update(const Vector v, const Dir &d, const Wall& w){
			curVec = v;
			curDir = d;
			if(state == IDOLE){
				state = SEARCHING_FOR_GOAL;
			}

			if(state == SEARCHING_FOR_GOAL){
				maze.updateWall(v, w);
				maze.updateStepMap(goal);
				step_t min_step = MAZE_STEP_MAX;
				for(Dir dir: d.ordered()){
					if(maze.getStep(v.next(dir))<min_step && !maze.getWall(v)[dir]){
						min_step = maze.getStep(v.next(dir));
						nextDir = dir;
					}
				}
				if(min_step == MAZE_STEP_MAX) state = GOT_LOST;
				if(std::find(goal.begin(), goal.end(),v)!=goal.end()){
					state = REACHED_GOAL;
					candidates = goal;
					candidates.erase(std::find(candidates.begin(), candidates.end(), v));
				}
			}

			if(state == REACHED_GOAL){
				maze.updateWall(v, w);
				maze.updateStepMap(candidates);
				step_t min_step = MAZE_STEP_MAX;
				for(dir_t dir: {0+d, 1+d, 3+d}){
					dir &= 3;
					if(maze.getStep(v.next(dir))<min_step && !maze.getWall(v)[dir]){
						min_step = maze.getStep(v.next(dir));
						nextDir = dir;
					}
				}
				candidates.erase(std::remove(candidates.begin(),candidates.end(), v), candidates.end());
				if(candidates.empty()){
					state = SEARCHING_ADDITIONALLY;
				}
			}

			if(state == SEARCHING_ADDITIONALLY){
				maze.updateWall(v, w);
				maze.updateStepMap({start});
				candidates.clear();
				for(int i=maze.getStep(v); i<maze.getStep(start); i--){
					for(dir_t dir: {0+d, 1+d, 3+d, 2+d}){
						Vector next = v.next(dir);
						if(maze.getStep(next) == i-1 && maze.getWall(next).nDone()!=4){
							candidates.push_back(next);
						}
					}
				}
				state = BACKING_TO_START;
			}

			if(state == BACKING_TO_START){
				maze.updateStepMap({start});
				step_t min_step = MAZE_STEP_MAX;
				for(dir_t dir: {0+d, 1+d, 3+d, 2+d}){
					dir &= 3;
					if(maze.getStep(v.next(dir))<min_step && !maze.getWall(v)[dir]){
						min_step = maze.getStep(v.next(dir));
						nextDir = dir;
					}
				}
				if(min_step == MAZE_STEP_MAX) state = GOT_LOST;
				if(v==start) {
					state = REACHED_START;
				}
			}
		}

		State getState(){
			return state;
		}
		Maze getMaze(){
			return maze;
		}
		dir_t getNextDir(){
			return nextDir;
		}
		Vector getCurVec(){
			return curVec;
		}
		dir_t getCurDir(){
			return curDir;
		}
		printInfo(int step){
			for(int i=0; i<MAZE_SIZE*2+4; i++) printf("\x1b[A");
			maze.printStepMap(curVec);
			printf("Step: %d, State: %s, Cur: (%d, %d, %d), Next Dir: %d\n", step, stateString(state), curVec.x, curVec.y, curDir, nextDir);
		}
	private:
		State state;
		Maze maze;
		const Vector start{0, 0};
		std::vector<Vector> goal;
		Vector curVec;
		dir_t curDir;
		dir_t nextDir;

		std::vector<Vector> candidates;
		/*
		   void calcNextDirMap(){
		   step_t min_step = MAZE_STEP_MAX;
		   for(dir_t dir: {0+d, 1+d, 3+d, 2+d}){
		   dir &= 3;
		   if(maze.getStep(v.next(dir))<min_step && !maze.getWall(v)[dir]){
		   min_step = maze.getStep(v.next(dir));
		   nextDir = dir;
		   }
		   }
		   if(min_step == MAZE_STEP_MAX) state = GOT_LOST;
		   if(std::any_of(goal.begin(), goal.end(), [&v](const auto &g){return g==v;})){
		   state = REACHED_GOAL;
		   candidates = goal;
		   candidates.erase(std::find(candidates.begin(), candidates.end(), v));
		   }
		   }
		   */
};

const char mazeData_fp2016[8+1][8+1] = {
	{"6beab6ab"},
	{"4aaa3c37"},
	{"c2ab4a1d"},
	{"b8a35683"},
	{"6a2954b5"},
	{"57575c29"},
	{"5549ca17"},
	{"dc8aaa9d"},
};

extern const char mazeData_maze[16+1][16+1] = {
	{"9551553ff9551553"},
	{"af92ffc556ffaffa"},
	{"a96aff939553affa"},
	{"8452ffaaa9568552"},
	{"affc53aaaa95693a"},
	{"effff86c6c2ffaaa"},
	{"9395569553c15286"},
	{"aaafff813ad43aaf"},
	{"aaefffac68556aaf"},
	{"a85153c556d556c3"},
	{"ae96fabff93ffffa"},
	{"a96d7aaffac53ffa"},
	{"869556affaff8552"},
	{"abafffc556ffaffa"},
	{"aaad515153ffaffa"},
	{"eec55456fc554556"},
};

int main(void){
	Maze sample(mazeData_maze,false);
	std::vector<Vector> goal = {Vector(7,7),Vector(7,8),Vector(8,8),Vector(8,7)};
	//Maze sample(mazeData_fp2016);
	//std::vector<Vector> goal = {Vector(7,7)};
	MazeAgent agent(goal);
	setvbuf(stdout, (char *)NULL, _IONBF, 0);
	agent.update(Vector(0, 0), 1, sample.getWall(0,0));
	agent.printInfo(0);
	usleep(100000);
	for(int step=1; ; step++){
	if(agent.getState() == MazeAgent::REACHED_START){
	printf("End\n");
	break;
	}
	if(agent.getState() == MazeAgent::GOT_LOST){
	printf("GOT LOST!\n");
	break;
	}
	dir_t nextDir = agent.getNextDir();
	Vector nextVec = agent.getCurVec().next(nextDir);
	agent.update(nextVec, nextDir, sample.getWall(nextVec));
	agent.printInfo(step);
	usleep(100000);
	}
	return 0;
}

