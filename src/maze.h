#pragma once

#include <queue>
#include <vector>
#include <array>
#include <algorithm>

#define MAZE_SIZE      16
#define MAZE_STEP_MAX  999

#if 1
#define C_RED     "\x1b[31m"
#define C_GREEN   "\x1b[32m"
#define C_YELLOW  "\x1b[33m"
#define C_BLUE    "\x1b[34m"
#define C_MAGENTA "\x1b[35m"
#define C_CYAN    "\x1b[36m"
#define C_RESET   "\x1b[0m"
#else
#define C_RED     ""
#define C_GREEN   ""
#define C_YELLOW  ""
#define C_BLUE    ""
#define C_MAGENTA ""
#define C_CYAN    ""
#define C_RESET   ""
#endif

#define DEEPNESS 0
#define SEARCHING_ADDITIALLY_AT_START 0

typedef uint16_t step_t;

class Dir{
	public:
		enum AbsoluteDir: int8_t { East, North, West, South, AbsMax };
		enum RelativeDir: int8_t { Forward, Left, Back, Right, RelMax };
		Dir(const enum AbsoluteDir d = East) : d(d) {}
		Dir(const int8_t d) : d(AbsoluteDir(d&3)) {}

		operator int8_t() const { return d; }
		inline const Dir operator=(const Dir& obj) { this->d = obj.d; return *this; }

		inline const Dir getRelative(const enum RelativeDir& rd) const { return Dir(rd-d); }
		inline const std::array<Dir, 4> ordered() const {
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

struct Vector{
	Vector(int8_t x=0, int8_t y=0) : x(x), y(y) {}
	Vector(const Vector& obj) : x(obj.x), y(obj.y) {}
	int8_t x, y;

	inline const Vector& operator=(const Vector& obj) { x=obj.x; y=obj.y; return *this; }
	inline const bool operator==(const Vector& obj) const { return x==obj.x && y==obj.y; }
	inline const bool operator!=(const Vector& obj) const { return x!=obj.x || y!=obj.y; }

	inline const Vector next(const Dir &dir) const {
		switch(dir){
			case Dir::East: return Vector(x+1, y);
			case Dir::North: return Vector(x, y+1);
			case Dir::West: return Vector(x-1, y);
			case Dir::South: return Vector(x, y-1);
		}
		printf("Warning: invalid direction\n");
		return *this;
	}
};

class Maze{
	public:
		Maze() { reset(); }
		Maze(const Maze& obj){ *this = obj; }
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
						updateWall(Vector(x, y), Dir::East, h&0x01);
						updateWall(Vector(x, y), Dir::North, h&0x02);
						updateWall(Vector(x, y), Dir::West, h&0x04);
						updateWall(Vector(x, y), Dir::South, h&0x08);
					}else{
						updateWall(Vector(x, y), Dir::East, h&0x02);
						updateWall(Vector(x, y), Dir::North, h&0x01);
						updateWall(Vector(x, y), Dir::West, h&0x08);
						updateWall(Vector(x, y), Dir::South, h&0x04);
					}
				}
		}
		const Maze& operator=(const Maze& obj){
			for(int8_t i=0; i<MAZE_SIZE-1; i++){
				wall[0][i]=obj.wall[0][i];
				wall[1][i]=obj.wall[1][i];
				known[0][i]=obj.known[0][i];
				known[1][i]=obj.known[1][i];
			}
			return *this;
		}
		void reset(){
			for(int8_t i=0; i<MAZE_SIZE-1; i++){
				wall[0][i]=0;
				wall[1][i]=0;
				known[0][i]=0;
				known[1][i]=0;
			}
			updateWall(Vector(0,0), Dir::East, true); //< start cell
			updateWall(Vector(0,0), Dir::North, false); //< start cell
		}
		bool isWall(const Vector& v, const Dir& d) const { return isWall(v.x, v.y, d); }
		bool isWall(const int8_t& x, const int8_t& y, const Dir& d) const {
			switch(d){
				case Dir::East:
					if(x<0 || x>MAZE_SIZE-2){ return true; }
					if(y<0 || y>MAZE_SIZE-1){ return true; }
					return wall[1][x] & (1<<y);
				case Dir::North:
					if(x<0 || x>MAZE_SIZE-1){ return true; }
					if(y<0 || y>MAZE_SIZE-2){ return true; }
					return wall[0][y] & (1<<x);
				case Dir::West:
					if(x-1<0 || x-1>MAZE_SIZE-2){ return true; }
					if(y<0 || y>MAZE_SIZE-1){ return true; }
					return wall[1][x-1] & (1<<y);
				case Dir::South:
					if(x<0 || x>MAZE_SIZE-1){ return true; }
					if(y-1<0 || y-1>MAZE_SIZE-2){ return true; }
					return wall[0][y-1] & (1<<x);
			}
			printf("Warning: invalid direction\n");
			return true;
		}
		void setWall(const Vector& v, const Dir& d, const bool& b) { return setWall(v.x, v.y, d, b); }
		void setWall(const int8_t& x, const int8_t& y, const Dir& d, const bool& b) {
			switch(d){
				case Dir::East:
					if(x<0 || x>MAZE_SIZE-2){ return; }
					if(y<0 || y>MAZE_SIZE-1){ return; }
					if(b) wall[1][x] |= (1<<y); else wall[1][x] &= ~(1<<y); return;
				case Dir::North:
					if(x<0 || x>MAZE_SIZE-1){ return; }
					if(y<0 || y>MAZE_SIZE-2){ return; }
					if(b) wall[0][y] |= (1<<x); else wall[0][y] &= ~(1<<x); return;
				case Dir::West:
					if(x-1<0 || x-1>MAZE_SIZE-2){ return; }
					if(y<0 || y>MAZE_SIZE-1){ return; }
					if(b) wall[1][x-1] |= (1<<y); else wall[1][x-1] &= ~(1<<y); return;
				case Dir::South:
					if(x<0 || x>MAZE_SIZE-1){ return; }
					if(y-1<0 || y-1>MAZE_SIZE-2){ return; }
					if(b) wall[0][y-1] |= (1<<x); else wall[0][y-1] &= ~(1<<x); return;
			}
		}
		bool isKnown(const Vector& v, const Dir& d) const { return isKnown(v.x, v.y, d); }
		bool isKnown(const int8_t& x, const int8_t& y, const Dir& d) const {
			switch(d){
				case Dir::East:
					if(x<0 || x>MAZE_SIZE-2){ return true; }
					if(y<0 || y>MAZE_SIZE-1){ return true; }
					return known[1][x] & (1<<y);
				case Dir::North:
					if(x<0 || x>MAZE_SIZE-1){ return true; }
					if(y<0 || y>MAZE_SIZE-2){ return true; }
					return known[0][y] & (1<<x);
				case Dir::West:
					if(x-1<0 || x-1>MAZE_SIZE-2){ return true; }
					if(y<0 || y>MAZE_SIZE-1){ return true; }
					return known[1][x-1] & (1<<y);
				case Dir::South:
					if(x<0 || x>MAZE_SIZE-1){ return true; }
					if(y-1<0 || y-1>MAZE_SIZE-2){ return true; }
					return known[0][y-1] & (1<<x);
			}
			printf("Warning: invalid direction\n");
			return false;
		}
		void setKnown(const Vector& v, const Dir& d, const bool& b) { return setKnown(v.x, v.y, d, b); }
		void setKnown(const int8_t& x, const int8_t& y, const Dir& d, const bool& b) {
			switch(d){
				case Dir::East:
					if(x<0 || x>MAZE_SIZE-2){ return; }
					if(y<0 || y>MAZE_SIZE-1){ return; }
					if(b) known[1][x] |= (1<<y); else known[1][x] &= ~(1<<y); return;
				case Dir::North:
					if(x<0 || x>MAZE_SIZE-1){ return; }
					if(y<0 || y>MAZE_SIZE-2){ return; }
					if(b) known[0][y] |= (1<<x); else known[0][y] &= ~(1<<x); return;
				case Dir::West:
					if(x-1<0 || x-1>MAZE_SIZE-2){ return; }
					if(y<0 || y>MAZE_SIZE-1){ return; }
					if(b) known[1][x-1] |= (1<<y); else known[1][x-1] &= ~(1<<y); return;
				case Dir::South:
					if(x<0 || x>MAZE_SIZE-1){ return; }
					if(y-1<0 || y-1>MAZE_SIZE-2){ return; }
					if(b) known[0][y-1] |= (1<<x); else known[0][y-1] &= ~(1<<x); return;
			}
		}
		bool canGo(const Vector& v, const Dir& d) const {
			return isKnown(v, d) && !isWall(v, d);
		}
		int8_t wallCount(const Vector& v) const {
			int8_t n=0;
			for(auto d: Dir::All()) if(isWall(v, d)) n++;
			return n;
			auto dirs = Dir::All();
			return std::count_if(dirs.begin(), dirs.end(), [&](const Dir& d){return isWall(v, d);});
		}
		int8_t knownCount(const Vector& v) const {
			auto dirs = Dir::All();
			return std::count_if(dirs.begin(), dirs.end(), [&](const Dir& d){return isKnown(v, d);});
		}
		void updateWall(const Vector& v, const Dir& d, const bool& b){
			setWall(v, d, b);
			setKnown(v, d, true);
		}
		void printWall(const step_t nums[MAZE_SIZE][MAZE_SIZE] = NULL, const Vector v = Vector(-1,-1)) const {
			printf("\n");
			for(int8_t y=MAZE_SIZE-1; y>=0; y--){
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					printf("+%s" C_RESET, isKnown(x,y,Dir::North) ? (isWall(x,y,Dir::North)?"---":"   ") : C_RED " - ");
				printf("+\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s" C_RESET, isKnown(x,y,Dir::West) ? (isWall(x,y,Dir::West)?"|":" ") : C_RED ":");
					if(nums!=NULL) printf("%s%3d" C_RESET, v==Vector(x,y)?C_YELLOW:C_CYAN, nums[y][x]);
					else printf("%s" C_RESET, v==Vector(x,y)?(C_YELLOW " X "):"   ");
				}
				printf("%s" C_RESET, isKnown(MAZE_SIZE-1,y,Dir::East) ? (isWall(MAZE_SIZE-1,y,Dir::East)?"|":" ") : C_RED ":");
				printf("\n");
			}
			for(uint8_t x=0; x<MAZE_SIZE; x++)
				printf("+%s" C_RESET, isKnown(x,0,Dir::South) ? (isWall(x,0,Dir::South)?"---":"   ") : C_RED " - ");
			printf("+\n");
		}
		void printPath(const Vector start, const std::vector<Dir>& dirs) const {
			step_t steps[MAZE_SIZE][MAZE_SIZE]={0};
			Vector v = start;
			int counter = 1;
			for(auto d: dirs){
				v = v.next(d);
				steps[v.y][v.x] = counter++;
			}
			printf("\n");
			for(int8_t y=MAZE_SIZE-1; y>=0; y--){
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					printf("+%s" C_RESET, isKnown(x,y,Dir::North) ? (isWall(x,y,Dir::North)?"---":"   ") : C_RED " - ");
				printf("+\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s" C_RESET, isKnown(x,y,Dir::West) ? (isWall(x,y,Dir::West)?"|":" ") : C_RED ":");
					if(steps[y][x]!=0) printf("%s%3d" C_RESET, C_YELLOW, steps[y][x]);
					else printf("%s", "   ");
				}
				printf("%s" C_RESET, isKnown(MAZE_SIZE-1,y,Dir::East) ? (isWall(MAZE_SIZE-1,y,Dir::East)?"|":" ") : C_RED ":");
				printf("\n");
			}
			for(uint8_t x=0; x<MAZE_SIZE; x++)
				printf("+%s" C_RESET, isKnown(x,0,Dir::South) ? (isWall(x,0,Dir::South)?"---":"   ") : C_RED " - ");
			printf("+\n");
		}
	private:
		uint32_t wall[2][MAZE_SIZE-1];
		uint32_t known[2][MAZE_SIZE-1];
};

