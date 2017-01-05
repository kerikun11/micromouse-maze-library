#include <cstdio>

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
	int8_t x;
	int8_t y;

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
		Wall wall[MAZE_SIZE][MAZE_SIZE];
		uint8_t stepMap[MAZE_SIZE][MAZE_SIZE];
};

int main(void){

	return 0;
}

