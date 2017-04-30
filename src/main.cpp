#include <cstdio>
#include <queue>
#include <vector>
#include <cstdint>

#define MAZE_SIZE		16
#define MAZE_STEP_MAX	999

#define EAST		0x01	// Wall
#define NORTH		0x02
#define WEST		0x04
#define SOUTH		0x08
#define D_EAST		0x10	// Done Wall
#define D_NOTTH		0x20
#define D_WEST		0x40
#define D_SOUTH		0x80
#define F_EAST		0x11	// Found Wall
#define F_NORTH		0x22
#define F_WEST		0x44
#define F_SOUTH		0x88
#define N_EAST		0x10	// Not Found Wall
#define N_NORTH		0x20
#define N_WEST		0x40
#define N_SOUTH		0x80

typedef int8_t dir_t;
typedef uint16_t step_t;

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
	inline int8_t nWall() const { return e+n+w+s; }
	inline int8_t nDone() const { return E+N+W+S; }

	inline void updateOne(dir_t dir, const bool b) {
		dir &= 3;
		if(b) flags |=   1<<dir;
		else  flags &= ~(1<<dir);
		flags |= 1<<(dir+4);
	}
	inline void updateAll(const uint8_t wall) {
		flags = 0xF0 | (wall&0x0F);
	}
	inline uint8_t rotate(dir_t dir) const {
		dir &= 3;
		return ((flags<<dir)|(flags>>(4-dir)))&0x0F;
	}
};

struct Vector{
	Vector(int8_t x=0, int8_t y=0) : x(x), y(y) {}
	int8_t x,y;

	inline const Vector& operator=(const Vector& obj) { x=obj.x; y=obj.y; return *this; }
	inline const Vector& operator+=(const Vector& obj) { x+=obj.x; y+=obj.y; return *this; }
	inline const Vector& operator-=(const Vector& obj) { x-=obj.x; y-=obj.y; return *this; }
	inline const Vector operator+(const Vector& obj) const { return Vector(x+obj.x, y+obj.y); }
	inline const Vector operator-(const Vector& obj) const { return Vector(x-obj.x, y-obj.y); }
	inline const bool operator==(const Vector& obj) const { return x==obj.x && y==obj.y; }
	inline const bool operator!=(const Vector& obj) const { return x!=obj.x || y!=obj.y; }
	inline const Vector next(const dir_t dir) const {
		switch(dir&0x3){
			case 0: return Vector(x+1, y);
			case 1: return Vector(x, y+1);
			case 2: return Vector(x-1, y);
			case 3: return Vector(x, y-1);
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
				getWall(i, 0) |= F_SOUTH;
				getWall(0, i) |= F_WEST;
				getWall(MAZE_SIZE-1, i) |= F_EAST;
				getWall(i, MAZE_SIZE-1) |= F_NORTH;
			}
			//* start cell
			updateWall(0,0,F_EAST|N_NORTH|F_WEST|F_SOUTH);
			//* initialize step map
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					getStep(x, y) = MAZE_STEP_MAX;
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
			getWall(v) = w;
			if(v.x==0) w.flags |= F_WEST;
			if(v.y==0) w.flags |= F_SOUTH;
			if(v.x==MAZE_SIZE-1) w.flags |= F_EAST;
			if(v.y==MAZE_SIZE-1) w.flags |= F_WEST;
			for(dir_t dir=0; dir<4; dir++){
				getWall(v.next(dir)).updateOne(dir+2, w[dir]);
			}
		}
		inline void printWall(step_t nums[MAZE_SIZE][MAZE_SIZE] = NULL){
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
				for(uint8_t x=0; x<MAZE_SIZE; x++) printf("+%s", wall[y][x].n ? "---" : "   ");
				printf("\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s", wall[y][x].w ? "|" : " ");
					if(nums==NULL) printf("   ");
					else printf("%3d", nums[y][x]);
				}
				printf("%s", wall[y][MAZE_SIZE-1].e ? "|" : " ");
				printf("\n");
			}
			for(uint8_t x=0; x<MAZE_SIZE; x++) printf("+%s", wall[0][x].s ? "---" : "   ");
			printf("\n");
			printf("\n");
#endif
		}
		inline void printPath(std::vector<Vector> path){
			step_t steps[MAZE_SIZE][MAZE_SIZE]={0};
			for(auto &v: path){
				steps[v.y][v.x] = (&v-&path[0]+1);
			}
			printWall(steps);
		}
		inline void printStepMap(){
			printWall(stepMap);
		}
		inline void updateStepMap(std::vector<Vector> dest){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					stepMap[y][x] = MAZE_STEP_MAX;
			for(auto d: dest) getStep(d) = 0;
			std::queue<Vector> q;
			for(auto d: dest) q.push(d);
			while(q.size()){
				Vector focus = q.front(); q.pop();
				uint16_t focus_step = getStep(focus);
				Wall focus_wall = getWall(focus);
				for(dir_t dir=0; dir<4; dir++){
					Vector next = focus.next(dir);
					if(!focus_wall[dir] && getStep(next)>focus_step+1){
						getStep(next) = focus_step+1;
						q.push(next);
					}
				}
			}
			printWall(stepMap);
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

		void update(const Vector v, const dir_t d, const Wall& w){
			curVec = v;
			curDir = d;
			if(state == IDOLE){
				state = SEARCHING_FOR_GOAL;
			}

			if(state == SEARCHING_FOR_GOAL){
				maze.updateWall(v, w);
				maze.updateStepMap(goal);
				step_t min_step = MAZE_STEP_MAX;
				for(dir_t dir: {0+d, 1+d, 3+d, 2+d}){
					dir &= 3;
					if(maze.getStep(v.next(dir))<min_step && !maze.getWall(v)[dir]){
						min_step = maze.getStep(v.next(dir));
						nextDir = dir;
					}
				}
				if(min_step == MAZE_STEP_MAX) state = GOT_LOST;
				for(auto g: goal){
					if(v.next(nextDir)==g) {
						state = REACHED_GOAL;
						list = goal;
						list.erase(list.begin()+(&g-&goal[0]));
					}
				}
			}

			if(state == REACHED_GOAL){
				maze.updateWall(v, w);
				maze.updateStepMap(list);
				step_t min_step = MAZE_STEP_MAX;
				for(int dir: {0+d, 1+d, 3+d}){
					dir &= 3;
					if(maze.getStep(v.next(dir))<min_step && !maze.getWall(v)[dir]){
						min_step = maze.getStep(v.next(dir));
						nextDir = dir;
					}
				}
				for(auto l: list){
					if(v.next(nextDir)==l) {
						state = REACHED_GOAL;
						list.erase(list.begin()+(&l-&list[0]));
						break;
					}
				}
				if(min_step == MAZE_STEP_MAX) state = GOT_LOST;
				if(list.empty()){
					state = SEARCHING_ADDITIONALLY;
				}
			}

			if(state == SEARCHING_ADDITIONALLY){
				state = BACKING_TO_START;
			}

			if(state == BACKING_TO_START){
				state = REACHED_START;
			}
			printf("State: %d, Cur: (%d, %d, %d), Next Dir: %d\n", state, curVec.x, curVec.y, curDir, nextDir);
		}

		State getState(){
			return state;
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

	private:
		State state;
		Maze maze;
		std::vector<Vector> goal;
		Vector curVec;
		dir_t curDir;
		dir_t nextDir;

		std::vector<Vector> list;
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
	Maze maze;
	//Maze sample(mazeData_fp2016);
	Maze sample(mazeData_maze,false);
	sample.printWall();
	std::vector<Vector> goal = {Vector(7,7),Vector(7,8),Vector(8,8),Vector(8,7)};
	MazeAgent agent(goal);

	agent.update(Vector(0, 0), 1, sample.getWall(0,0));
	while(1){
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
	}
	maze.printStepMap();
	return 0;
}

