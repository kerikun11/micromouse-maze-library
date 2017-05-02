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
			//* step map
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					stepMap[y][x] = 0;
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
		inline void printWall(const step_t nums[MAZE_SIZE][MAZE_SIZE] = NULL, const Vector v = Vector(-1,-1)) const {
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
		inline void printPath(std::vector<Vector> path) const {
			step_t steps[MAZE_SIZE][MAZE_SIZE]={0};
			for(auto &v: path){
				steps[v.y][v.x] = (&v-&path[0]+1);
			}
			printWall(steps);
		}
		inline void printStepMap(const Vector v=Vector(-1,-1)) const {
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

		void update(const Vector v, const Dir &dir, const Wall& w){
			curVec = v;
			curDir = dir;
			maze.updateWall(v, w);
		}
		bool calcNextDir(){
			if(state == IDOLE){
				state = SEARCHING_FOR_GOAL;
			}

			if(state == SEARCHING_FOR_GOAL){
				if(std::find(goal.begin(), goal.end(), curVec)!=goal.end()){
					state = REACHED_GOAL;
					candidates = goal;
				}else{
					maze.updateStepMap(goal);
					calcNextDirByStepMap();
				}
			}

			if(state == REACHED_GOAL){
				candidates.erase(std::find(candidates.begin(),candidates.end(), curVec));
				if(candidates.empty()){
					state = SEARCHING_ADDITIONALLY;
				}else{
					maze.updateStepMap(candidates);
					calcNextDirByStepMap();
				}
			}

			if(state == SEARCHING_ADDITIONALLY){
				maze.updateStepMap({start});
				candidates.clear();
				std::vector<step_t> goal_steps;
				for(auto g:goal)goal_steps.push_back(maze.getStep(g));
				step_t goal_step = *(min_element(goal_steps.begin(), goal_steps.end()));
				for(int i=0; i<MAZE_SIZE; i++){
					for(int j=0; j<MAZE_SIZE; j++){
						//if(maze.getWall(i,j).nDone()!=4 && maze.getStep(i,j) != MAZE_STEP_MAX){
						if(maze.getWall(i,j).nDone()!=4 && maze.getStep(i,j) < goal_step){
							candidates.push_back(Vector(i,j));
						}
					}
				}
				if(candidates.empty()){
					state = BACKING_TO_START;
				}else{
					maze.updateStepMap(candidates);
					calcNextDirByStepMap();
				}
			}

			if(state == BACKING_TO_START){
				if(curVec==start) {
					state = REACHED_START;
				}else{
					maze.updateStepMap({start});
					calcNextDirByStepMap();
				}
			}

			return true;
		}

		State getState() const {
			return state;
		}
		Maze getMaze() const {
			return maze;
		}
		Dir getNextDir() const {
			return nextDir;
		}
		Vector getCurVec() const {
			return curVec;
		}
		Dir getCurDir() const {
			return curDir;
		}
		void printInfo(int step) const {
			for(int i=0; i<MAZE_SIZE*2+4; i++) printf("\x1b[A");
			maze.printStepMap(curVec);
			printf("Step: %d, State: %s, Cur: (%d, %d, %d), Next Dir: %d      \n",
					step, stateString(state), curVec.x, curVec.y, uint8_t(curDir), uint8_t(nextDir));
		}
	private:
		State state;
		Maze maze;
		const Vector start{0, 0};
		std::vector<Vector> goal;
		Vector curVec;
		Dir curDir;
		Dir nextDir;

		std::vector<Vector> candidates;
		void calcNextDirByStepMap(){
			step_t min_step = MAZE_STEP_MAX;
			for(Dir d: curDir.ordered()){
				if(maze.getStep(curVec.next(d))<min_step && !maze.getWall(curVec)[d]){
					min_step = maze.getStep(curVec.next(d));
					nextDir = d;
				}
			}
			if(min_step == MAZE_STEP_MAX) state = GOT_LOST;
		}
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

extern const char mazeData_maze2013exp[16+1][16+1] = {
		{"9795555555551393"},
		{"856915555553eaaa"},
		{"8796a95153d43c6a"},
		{"ad056ad07a93853a"},
		{"ad0796d07c6aad2a"},
		{"a943c3d0793ac3aa"},
		{"a8543ad056ac3aaa"},
		{"ac53ac38396baaaa"},
		{"a956a96c6c3c2aaa"},
		{"ac53c43939696aaa"},
		{"a95693c6c6bad2aa"},
		{"a8556a9153c296aa"},
		{"a8393c6c5296abaa"},
		{"aac681793c43a86a"},
		{"aabbec56c5546ad2"},
		{"ec44555555555456"},
};

extern const char mazeData_maze2013fr[16+1][16+1] = {
		{"9115151553ff9113"},
		{"aaafafaf94556aaa"},
		{"a8696fafa95556aa"},
		{"82fad543aa95556a"},
		{"aa92fffac6c55392"},
		{"a8681516f95556aa"},
		{"c2faafa954553faa"},
		{"f816afa83953afaa"},
		{"fac3856c6afaafaa"},
		{"92fac5553c3ac56a"},
		{"ac54539543ac5552"},
		{"affffaa93aaf9552"},
		{"8515542aac696952"},
		{"af851546c3fafafa"},
		{"afafaf9552fafafa"},
		{"efc5456ffc545456"},
};

extern const char mazeData_maze3[16+1][16+1] = {
		{"d5553fffffffffff"},
		{"d5116fff93ffffff"},
		{"ffe815556affffff"},
		{"fffeaf93fa93ffff"},
		{"ff95052afaaaffff"},
		{"ffc52baa96aaffff"},
		{"ff956c6c056c5553"},
		{"9507fff92ffffffa"},
		{"a96f955443fffffa"},
		{"aafbaffff8553ffa"},
		{"aef86ffffaffc156"},
		{"c53afffffafffaff"},
		{"b96a955552fffaff"},
		{"86beefbffafffaff"},
		{"8545156ffc5556fb"},
		{"efffeffffffffffe"},
};

extern const char mazeData_maze4[16+1][16+1] = {
		{"d51157f9515557d3"},
		{"97ac5552fc55153a"},
		{"afaff97ad153afaa"},
		{"c5413c52fad6c3c2"},
		{"fbfaabbc56f956fa"},
		{"d452ac053ffaf956"},
		{"d13aad6f8156d453"},
		{"faac2d392c39517a"},
		{"fc43afac47aefafa"},
		{"93bc43af9383fa96"},
		{"aac552c56c6a946b"},
		{"ac553c5555568552"},
		{"afffabffb9556fba"},
		{"affd04154695512a"},
		{"83938501552ffeea"},
		{"ec6c6feeffc55556"},
};

extern const char mazeData_maze5[16+1][16+1] = {
		{"f93f953bfd397d53"},
		{"d46b852ed146fbbe"},
		{"f93c4507babbd02b"},
		{"feef97ed6a807e86"},
		{"d17be97d546c3d6f"},
		{"febc383b9117c57f"},
		{"d52d2eea86c7fd13"},
		{"ffe941502d57d506"},
		{"d796fc3c2bd15107"},
		{"f92b97c52ed47ec7"},
		{"d2c4417d693fbbff"},
		{"d4517ad392c7eabb"},
		{"fbbc1456c6ff9406"},
		{"9443ad13d795456f"},
		{"af942faa914553bf"},
		{"efed6feeec55546f"},
};

int main(void){
	setvbuf(stdout, (char *)NULL, _IONBF, 0);
	//Maze sample(mazeData_maze, false);
	Maze sample(mazeData_maze3, false);
	std::vector<Vector> goal = {Vector(7,7),Vector(7,8),Vector(8,8),Vector(8,7)};
	//Maze sample(mazeData_fp2016);
	//std::vector<Vector> goal = {Vector(7,7)};
	MazeAgent agent(goal);
	agent.update(Vector(0, 0), 1, sample.getWall(0,0));
	for(int step=1; ; step++){
		agent.calcNextDir();
		if(agent.getState() == MazeAgent::GOT_LOST){
			printf("GOT LOST!\n");
			break;
		}
		Dir nextDir = agent.getNextDir();
		Vector nextVec = agent.getCurVec().next(nextDir);
		// move robot here
		agent.printInfo(step);
		usleep(100000);
		agent.update(nextVec, nextDir, sample.getWall(nextVec));
		MazeAgent::State state = agent.getState();
		if(agent.getState() == MazeAgent::REACHED_START){
			printf("End\n");
			break;
		}
	}
	return 0;
}

