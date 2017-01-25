#include <cstdio>
#include <queue>

#define MAZE_SIZE	5

#define EAST		0x01
#define NORTH		0x02
#define WEST		0x04
#define SOUTH		0x08
#define D_EAST		0x10
#define D_NORTH		0x20
#define D_WEST		0x40
#define D_SOUTH		0x80
#define F_EAST		0x11
#define F_NORTH		0x22
#define F_WEST		0x44
#define F_SOUTH		0x88
#define N_EAST		0x10
#define N_NORTH		0x20
#define N_WEST		0x40
#define N_SOUTH		0x80

union Wall{
	Wall(uint8_t value = 0) : flags(value) {}
	uint8_t flags;
	struct{
		uint8_t e:1;	// east
		uint8_t n:1;	// north
		uint8_t w:1;	// west
		uint8_t s:1;	// south
		uint8_t E:1;	// done east
		uint8_t N:1;	// done north
		uint8_t W:1;	// done west
		uint8_t S:1;	// done south
	};
	inline operator uint8_t() const { return flags; }
	inline uint8_t operator[](uint8_t index) const { return (flags>>index)&0x01; }
	inline uint8_t set(uint8_t index) { flags |= 1<<index; }
	inline uint8_t clear(uint8_t index) { flags &= ~(1<<index); }
	inline uint8_t operator=(const Wall& obj) { flags=obj.flags; return flags; }
	inline uint8_t operator|=(const Wall& obj) { flags|=obj.flags; return flags; }
	inline uint8_t operator&=(const Wall& obj) { flags&=obj.flags; return flags; }
	inline int8_t nWall() { return e+n+w+s; }
	inline int8_t nDone() { return E+N+W+S; }
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
	inline const Vector next(const int8_t dir) const {
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
		inline void reset(){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					getWall(x,y) = 0x00;
			for(int i=0; i<MAZE_SIZE; i++){
				getWall(i, 0) |= F_SOUTH;
				getWall(0, i) |= F_WEST;
				getWall(MAZE_SIZE-1, i) |= F_EAST;
				getWall(i, MAZE_SIZE-1) |= F_NORTH;
			}
			updateWall(0,0,F_EAST|N_NORTH|F_WEST|F_SOUTH);
		}
		inline Wall& getWall(const Vector& v) { return getWall(v.x, v.y); }
		inline Wall& getWall(const int x, const int y) {
			static Wall edge;
			edge.flags = 0xFF;
			if(x<0 || y<0 || x>MAZE_SIZE-1 || y>MAZE_SIZE-1) return edge;
			return wall[y][x];
		}
		inline uint8_t& getStep(const int x, const int y) { return stepMap[y][x]; }
		inline uint8_t& getStep(const Vector& v) { return stepMap[v.y][v.x]; }
		inline void updateWall(const int x, const int y, const Wall& w){ return updateWall(Vector(x,y),w); }
		inline void updateWall(const Vector& v, const Wall& w){
			getWall(v) = w;
			for(int dir=0; dir<4; dir++){
				Wall& next = getWall(v.next(dir));
				next |= ((0x11<<dir)&w)<<((dir+2)&0x3);
			}
		}
		inline void printWall(uint8_t nums[MAZE_SIZE][MAZE_SIZE] = NULL){
			for(int16_t y=MAZE_SIZE-1; y>=0; y--){
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
			printf("\n");
		}
		inline void updateStepMap(const Vector& dest){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					stepMap[y][x] = 0xff;
			getStep(dest) = 0;
			std::queue<Vector> q;
			q.push(dest);
			while(q.size()){
				Vector focus = q.front(); q.pop();
				uint8_t focus_step = getStep(focus);
				Wall focus_wall = getWall(focus);
				for(int dir=0; dir<4; dir++){
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
		uint8_t stepMap[MAZE_SIZE][MAZE_SIZE];
};

int main(void){
	Maze maze;
	Vector goal(2, 2);
	uint8_t nWall[MAZE_SIZE][MAZE_SIZE];
	for(int x=0; x<MAZE_SIZE; x++)
		for(int y=0; y<MAZE_SIZE; y++)
			nWall[y][x] = maze.getWall(x,y).nDone();
	maze.printWall(nWall);
	maze.updateWall(Vector(0, 0), F_EAST|N_NORTH|F_WEST|F_WEST);
	maze.updateStepMap(goal);
	return 0;
}

