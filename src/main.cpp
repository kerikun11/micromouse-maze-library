#include <cstdio>
#include <cstdint>
#include <queue>
#include <vector>
#include <array>
#include <algorithm>
#include <unistd.h>

#define MAZE_SIZE      8
#define MAZE_STEP_MAX  999

#define C_RED     "\x1b[31m"
#define C_GREEN   "\x1b[32m"
#define C_YELLOW  "\x1b[33m"
#define C_BLUE    "\x1b[34m"
#define C_MAGENTA "\x1b[35m"
#define C_CYAN    "\x1b[36m"
#define C_RESET   "\x1b[0m"

#define DEEPNESS 0
#define SEARCHING_ADDITIALLY_AT_START 1
#define DISPLAY 1

typedef uint16_t step_t;

class Dir{
	public:
		enum AbsoluteDir: int8_t { East, North, West, South, AbsMax };
		enum RelativeDir: int8_t { Forward, Left, Back, Right, RelMax };
		Dir(const enum AbsoluteDir d = East) : d(d) {}
		Dir(const int8_t d) : d(AbsoluteDir(d&3)) {}

		operator int8_t() const { return d; }
		inline const Dir operator=(const Dir& obj) { this->d = obj.d; return *this; }
		inline const Dir operator-(const Dir& obj) const { return Dir(d-obj.d); }

		inline const Dir getRelative(const enum RelativeDir& rd) const { return Dir(rd-d); }
		const std::array<Dir, 4> ordered() const {
			std::array<Dir, 4> order{d, d+1, d+3, d+2};
			return order;
		}
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

	inline int8_t nWall() const { return e+n+w+s; }
	inline int8_t nKnown() const { return E+N+W+S; }

	inline void updateOne(const Dir &dir, const bool b) {
		if(b) flags |=   1<<dir;
		else  flags &= ~(1<<dir);
		flags |= 1<<(dir+4);
	}
	inline void updateAll(const uint8_t wall) {
		flags = 0xF0 | (wall&0x0F);
	}

	inline bool canGoDir(const Dir& d) const {
		return !(flags&(1<<d)) && (flags&(1<<(4+d)));
	}

	inline uint8_t rotate(const Dir &dir) const {
		return ((flags<<dir)|(flags>>(4-dir)))&0x0F;
	}
};

struct Vector{
	Vector(int8_t x=0, int8_t y=0) : x(x), y(y) {}
	Vector(const Vector& obj) : x(obj.x), y(obj.y) {}
	int8_t x,y;

	inline const Vector& operator=(const Vector& obj) { x=obj.x; y=obj.y; return *this; }
	inline const Vector& operator+=(const Vector& obj) { x+=obj.x; y+=obj.y; return *this; }
	inline const Vector& operator-=(const Vector& obj) { x-=obj.x; y-=obj.y; return *this; }
	inline const Vector operator+(const Vector& obj) const { return Vector(x+obj.x, y+obj.y); }
	inline const Vector operator-(const Vector& obj) const { return Vector(x-obj.x, y-obj.y); }
	inline const bool operator==(const Vector& obj) const { return x==obj.x && y==obj.y; }
	inline const bool operator!=(const Vector& obj) const { return x!=obj.x || y!=obj.y; }

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
		const Maze& operator=(Maze& obj){
			for(int8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					getWall(x, y) = obj.getWall(x, y);
					getStep(x, y, 0) = obj.getStep(x, y, 0);
					getStep(x, y, 1) = obj.getStep(x, y, 1);
				}
			return *this;
		}
		inline void reset(){
			//* clear all wall
			for(int8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					getWall(x, y) = 0;
					getStep(x, y, 0) = 0;
					getStep(x, y, 1) = 0;
				}
			//* set outline
			for(int8_t i=0; i<MAZE_SIZE; i++){
				getWall(i, 0).updateOne(Dir::South, true);
				getWall(0, i).updateOne(Dir::West, true);
				getWall(MAZE_SIZE-1, i).updateOne(Dir::East, true);
				getWall(i, MAZE_SIZE-1).updateOne(Dir::North, true);
			}
			//* start cell
			updateWall(0,0,0x0b);
		}
		inline Wall& getWall(const Vector& v) { return getWall(v.x, v.y); }
		inline Wall& getWall(const int8_t x, const int8_t y) {
			static Wall edge;
			edge.flags = 0xFF;
			if(x<0 || y<0 || x>MAZE_SIZE-1 || y>MAZE_SIZE-1) return edge;
			return wall[y][x];
		}
		inline step_t& getStep(const Vector& v, const uint8_t nthMap = 0) { return getStep(v.x, v.y, nthMap); }
		inline step_t& getStep(const int8_t x, const int8_t y, const uint8_t nthMap = 0) {
			static step_t outside;
			outside = MAZE_STEP_MAX;
			if(x<0 || y<0 || x>MAZE_SIZE-1 || y>MAZE_SIZE-1){
				printf("Warning: pefered out of field ------------------------------------------> %2d, %2d\n", x, y);
				return outside;
			}
			return stepMap[nthMap][y][x];
		}
		inline void updateWall(const int8_t x, const int8_t y, const Wall& w){ return updateWall(Vector(x, y), w); }
		inline void updateWall(const Vector& v, Wall w){
			if(v.x==0) w.updateOne(Dir::West, true);
			if(v.y==0) w.updateOne(Dir::South, true);
			if(v.x==MAZE_SIZE-1) w.updateOne(Dir::East, true);
			if(v.y==MAZE_SIZE-1) w.updateOne(Dir::North, true);
			getWall(v).updateAll(w);
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
			printf("+\n");
#endif
		}
		inline void printPath(std::vector<Vector> path) const {
			step_t steps[MAZE_SIZE][MAZE_SIZE]={0};
			for(auto &v: path){
				steps[v.y][v.x] = (&v-&path[0]+1);
			}
			printf("\n");
			for(int8_t y=MAZE_SIZE-1; y>=0; y--){
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					printf("+%s" C_RESET, wall[y][x].N ? (wall[y][x].n?"---":"   ") : C_RED " - ");
				printf("+\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s" C_RESET, wall[y][x].W ? (wall[y][x].w?"|":" ") : C_RED ":");
					auto it = std::find(path.begin(), path.end(), Vector(x,y));
					if(it!=path.end()) printf("%s%3d" C_RESET, C_YELLOW, it-path.begin());
					else printf("%s", "   ");
				}
				printf("%s" C_RESET, wall[y][MAZE_SIZE-1].E ? (wall[y][MAZE_SIZE-1].e?"|":" ") : C_RED ":");
				printf("\n");
			}
			for(uint8_t x=0; x<MAZE_SIZE; x++)
				printf("+%s" C_RESET, wall[0][x].S ? (wall[0][x].s?"---":"   ") : C_RED " - ");
			printf("+\n");
		}
		inline void printStepMap(const Vector v=Vector(-1,-1), const uint8_t nthMap = 0) const {
			printWall(stepMap[nthMap], v);
		}
		inline void updateStepMap(const std::vector<Vector> dest, const uint8_t nthMap = 0){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					getStep(x, y, nthMap) = MAZE_STEP_MAX;
			std::queue<Vector> q;
			for(auto d: dest) {
				getStep(d, nthMap) = 0;
				q.push(d);
			}
			while(!q.empty()){
				Vector focus = q.front(); q.pop();
				step_t focus_step = getStep(focus, nthMap);
				Wall focus_wall = getWall(focus);
				for(Dir dir: Dir::All()){
					Vector next = focus.next(dir);
					if(!focus_wall[dir] && getStep(next, nthMap)>focus_step+1){
						getStep(next, nthMap) = focus_step+1;
						q.push(next);
					}
				}
			}
		}
	private:
		Wall wall[MAZE_SIZE][MAZE_SIZE];
		step_t stepMap[2][MAZE_SIZE][MAZE_SIZE];
};

class MazeAgent{
	public:
		MazeAgent(const std::vector<Vector>& goal){
			reset(goal);
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
		void reset(const std::vector<Vector>& goal){
			this->goal = goal;
			maze.reset();
			curVec = Vector(0, 0);
			state = IDOLE;
		}
		void forceBackToStart(){
			state = BACKING_TO_START;
		}
		void updateAll(const Vector& v, const Dir& dir, const Wall& w){
			curVec = v;
			curDir = dir;
			maze.updateWall(v, w);
		}
		void updateCurVec(const Vector& v){ curVec = v; }
		void updateCurDir(const Dir& d){ curDir = d; }
		void updateWall(const Vector& v, const Wall& w){ maze.updateWall(v, w); }
		bool calcNextDir(){
			State prev_state = getState();
			if(state == IDOLE){
				step=0; f=0; l=0; r=0; b=0;
				state = SEARCHING_FOR_GOAL;
#if SEARCHING_ADDITIALLY_AT_START
				state = SEARCHING_ADDITIONALLY;
#endif
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
				maze.updateStepMap(goal, 1);
				candidates.clear();
				std::vector<step_t> goal_steps;
				for(auto g:goal) goal_steps.push_back(maze.getStep(g));
				step_t goal_step = *(std::min_element(goal_steps.begin(), goal_steps.end()));
				for(int i=0; i<MAZE_SIZE; i++){
					for(int j=0; j<MAZE_SIZE; j++){
						Vector v(i,j);
#if DEEPNESS == 0
						if(maze.getWall(i,j).nKnown()!=4 && (maze.getStep(i,j)+maze.getStep(i,j,1)) <= goal_step){
							candidates.push_back(v);
						}
#elif DEEPNESS == 1
						if(maze.getWall(i,j).nKnown()!=4 && (maze.getStep(i,j)) <= goal_step){
							candidates.push_back(v);
						}
#elif DEEPNESS == 2
						if(maze.getWall(i,j).nKnown()!=4 && (maze.getStep(i,j)) != MAZE_STEP_MAX){
							candidates.push_back(v);
						}
#endif
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
				maze.updateStepMap({start});
				calcNextDirByStepMap();
				if(curVec.next(nextDirs.back())==start) {
					state = REACHED_START;
				}
			}
			for(auto d: nextDirs){
				step++;
				f += curDir.getRelative(Dir::Forward) == d;
				l += curDir.getRelative(Dir::Left   ) == d;
				r += curDir.getRelative(Dir::Right  ) == d;
				b += curDir.getRelative(Dir::Back   ) == d;
			}
			if(getState() != prev_state) return true;
			return false;
		}

		bool calcShortestPath(){
			maze.updateStepMap(goal);
			shortestPath.clear();
			Vector v = start;
			Dir dir = Dir::North;
			shortestPath.push_back(v);
			while(1){
				auto dirs = dir.ordered();
				auto it = std::find_if(dirs.begin(), dirs.end(),[&](auto d){
						if(!maze.getWall(v).canGoDir(d)) return false;
						return maze.getStep(v.next(d)) == maze.getStep(v)-1;
						});
				if(it == dirs.end()) return false;
				dir = *it;
				v=v.next(dir);
				shortestPath.push_back(v);
				if(maze.getStep(v)==0) break;
			}
			return true;
		}

		State getState() const {
			return state;
		}
		Maze getMaze() const {
			return maze;
		}
		std::vector<Dir> getNextDirs() const {
			return nextDirs;
		}
		Vector getCurVec() const {
			return curVec;
		}
		Dir getCurDir() const {
			return curDir;
		}
		const std::vector<Vector>& getShortestPath() const {
			return shortestPath;
		}
		void printInfo() const {
			for(int i=0; i<MAZE_SIZE*2+4; i++) printf("\x1b[A");
			maze.printStepMap(curVec);
			printf("Cur: ( %3d, %3d, %3d), State: %s       \n", curVec.x, curVec.y, uint8_t(curDir), stateString(state));
			printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d\n", step, f, l, r, b);
		}
		void printPath() const {
			for(int i=0; i<MAZE_SIZE*2+5; i++) printf("\x1b[A");
			maze.printPath(shortestPath);
			printf("\n\n\n");
			printf("Shortest Step: %d\n", shortestPath.size()-1);
		}
	private:
		State state;
		Maze maze;
		const Vector start{0, 0};
		std::vector<Vector> goal;
		Vector curVec;
		Dir curDir;
		std::vector<Dir> nextDirs;
		int step=0,f=0,l=0,r=0,b=0;
		std::vector<Vector> shortestPath;

		std::vector<Vector> candidates;
		void calcNextDirByStepMap(){
			nextDirs.clear();
			Vector focus_v = curVec;
			Dir focus_d = curDir;
			while(1){
				auto dirs = focus_d.ordered();
				auto it = std::find_if(dirs.begin(), dirs.end(), [&](auto d){
						if(!maze.getWall(focus_v).canGoDir(d))return false;
						return maze.getStep(focus_v.next(d)) == maze.getStep(focus_v)-1;
						});
				if(it==dirs.end()) break;
				nextDirs.push_back(*it);
				focus_d = *it;
				focus_v = focus_v.next(*it);
			}
			if(nextDirs.empty()) state = GOT_LOST;
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

const char mazeData_maze2013half[32+1][32+1] = {
	{"95555115555555395555555395555393"},
	{"a9153aa9515153aa9515153aa955382a"},
	{"aa816aac16bc16aac16bc16ac417aaaa"},
	{"a82816c16943c16c16943c3a9569442a"},
	{"aa86c396943c3c396945456c4514396a"},
	{"a8053c6947a96fc692fffffffd052c3a"},
	{"82852954556c5553aafffffffd05296a"},
	{"a8052a955539553aaafffffffd052c3a"},
	{"86c56aa9556c53aaaafffffffd056d2a"},
	{"c5553c6c555556aaaafffffffd0793aa"},
	{"d55385555515556aaafffffffd07ac6a"},
	{"913aafffffa95556aa9555555507c53a"},
	{"aaaaafffffaa95556ac53d515507956a"},
	{"aaaaafffffaaa9555295695055078552"},
	{"aaaaafffffaaaa9552c538545507853a"},
	{"aaaaafffffaaaa85545568551507afaa"},
	{"aaaaafffffaaaac5395554554547c56a"},
	{"aaaaafffffaaaa93aa95555555555552"},
	{"aac6afffffac6aac6aa955555555553a"},
	{"ac554555516d12affaaa9555555553aa"},
	{"8155155514796ac552aaaffffff93aaa"},
	{"a83943f9695454553aaaaffffffaaaaa"},
	{"82841696bc539553aaaaaffffffaaaaa"},
	{"ac4141456956a93aaaaaaffffffaaaaa"},
	{"853c16913c53aac46aaaaffffffaaaaa"},
	{"a94143802956ac5556aaaffffffaaaaa"},
	{"ac1416846c53855553aaaffffffaaaaa"},
	{"a94143839156c1553aaac5555556aaaa"},
	{"841416ac40553c156aac555555556aaa"},
	{"a941438554156d4152c55555555556aa"},
	{"805452c555455554545555555555556a"},
	{"ec555455555555555555555555555556"},
};

int main(void){
	setvbuf(stdout, (char *)NULL, _IONBF, 0);

#if MAZE_SIZE == 8
	std::vector<Vector> goal = {Vector(7,7)};
	Maze sample(mazeData_fp2016);
#elif MAZE_SIZE == 16
	std::vector<Vector> goal = {Vector(7,7),Vector(7,8),Vector(8,8),Vector(8,7)};
	//Maze sample(mazeData_maze, false);
	Maze sample(mazeData_maze, false);
#elif MAZE_SIZE == 32
	std::vector<Vector> goal = {Vector(7,7)};
	Maze sample(mazeData_maze2013half, false);
#endif

	MazeAgent agent(goal);
	agent.updateAll(Vector(0, 0), 1, sample.getWall(0,0));
	while(1){
		agent.calcNextDir();
		if(agent.getState() == MazeAgent::GOT_LOST){
			printf("GOT LOST!\n");
			break;
		}
		for(Dir nextDir: agent.getNextDirs()){
#if DISPLAY
			usleep(100000);
			agent.printInfo();
#endif
			Vector nextVec = agent.getCurVec().next(nextDir);
			// move robot here
			agent.updateCurDir(nextDir);
			agent.updateCurVec(nextVec);
		}
		Wall found_wall = sample.getWall(agent.getCurVec());
		agent.updateWall(agent.getCurVec(), found_wall);
		if(agent.getState() == MazeAgent::REACHED_START){
			break;
		}
#if DISPLAY
		usleep(500000);
#endif
	}
	agent.printInfo();
	sleep(1);
	if(!agent.calcShortestPath()){
		printf("Failed to find shortest path!\n");
	}
	agent.printPath();
	printf("End\n");
	return 0;
}

