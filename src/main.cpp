#include <cstdio>
#include <queue>

#define MAZE_SIZE	16

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
	inline uint8_t operator=(Wall& obj) { flags=obj.flags; return flags; }

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
	inline const Vector operator[](const int8_t dir) const {
		switch(dir&0x3){
			case 0:
				return Vector(x+1, y);
			case 1:
				return Vector(x, y+1);
			case 2:
				return Vector(x-1, y);
			case 3:
				return Vector(x, y-1);
		}
	}
};

class Maze{
	public:
		Maze(){}
		inline Wall& getWall(const int x, const int y) { return wall[y][x]; }
		inline Wall& getWall(const Vector& v) { return wall[v.y][v.x]; }
		inline uint8_t& getStepMap(const int x, const int y) { return stepMap[y][x]; }
		inline uint8_t& getStepMap(const Vector& v) { return stepMap[v.y][v.x]; }
		inline void updateWall(const uint8_t x, const uint8_t y, const uint8_t w){
			wall[y][x] |= w;
			for(int dir=0; dir<4; dir++)
			if(x != 0          ) wall[y][x-1] |= (w&F_WEST )>>2; //< Not Westernmost
			if(x != MAZE_SIZE-1) wall[y][x+1] |= (w&F_EAST )<<2; //< Not Easternmost
			if(y != 0          ) wall[y-1][x] |= (w&F_SOUTH)>>2; //< Not Southernmost
			if(y != MAZE_SIZE-1) wall[y+1][x] |= (w&F_NORTH)<<2; //< Not Northernmost
		}
		inline void updateWall(const Vector& v, const uint8_t w) { return updateWall(v.x, v.y, w);}
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
			getStepMap(dest) = 0;
			std::queue<Vector> q;
			q.push(dest);
			while(q.size()){
				Vector focus = q.front(); q.pop();
				uint8_t focus_step = getStepMap(focus);
				uint8_t focus_wall = getWall(focus);
				for(int dir=0; dir<4; dir++){
					Vector& next = focus[dir];
					if(!focus_wall[dir] && getStepMap(next)>focus_step+1){
						getStepMap(next) = focus_step+1;
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

	return 0;
}

