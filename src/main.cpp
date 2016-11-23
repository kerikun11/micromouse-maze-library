#include <cstdio>

#define MAZE_SIZE	5

class MazeWall {
	public:
		union{
			uint8_t byte;
			struct{
				uint8_t North:1;
				uint8_t East:1;
				uint8_t South:1;
				uint8_t West:1;
				uint8_t DoneNorth:1;
				uint8_t DoneEast:1;
				uint8_t DoneSouth:1;
				uint8_t DoneWest:1;
			}bits;
		};
		MazeWall(uint8_t byte=0) : byte(byte) {}

		inline operator uint8_t() const { return byte; }
		inline uint8_t operator|(uint8_t value) const { return byte | value; }
		inline uint8_t operator&(uint8_t value) const { return byte & value; }
		inline uint8_t operator=(uint8_t value) { return byte = value; }
		inline uint8_t operator|=(uint8_t value) { return byte |= value; }
		inline uint8_t operator&=(uint8_t value) { return byte &= value; }
		inline uint8_t operator=(MazeWall &obj) { return byte = obj.byte; }
		inline uint8_t operator[](uint8_t index) const {return (byte & (0x01<<index)) ? 1:0; }

		inline void update(uint8_t index, uint8_t value){
			if(value) byte |= (0x01<<index);
			else byte &= ~(0x01<<index);
		}
		inline bool isDoneAll() const { return (byte | 0x0f) == 0xff; }
};

class MazePosition {
	public:
		MazePosition(uint8_t x=0, uint8_t y=0, uint8_t dir=0) : x(x), y(y), dir(dir) {}
		uint8_t x;
		uint8_t y;
		uint8_t dir;
};

class Maze {
	public:
		Maze(){
			reset();
		}
		const Maze& operator=(const Maze &obj){
			for (int i=0;i<MAZE_SIZE;i++) {
				for (int j=0;j<MAZE_SIZE;j++) {
					_wall[i][j] = obj._wall[i][j];
					stepMap[i][j] = obj.stepMap[i][j];
				}
			}
			return *this;
		}
		void reset(){
			for(int i=0; i<MAZE_SIZE; i++){
				_wall[i][MAZE_SIZE-1].bits.East=1;
				_wall[i][MAZE_SIZE-1].bits.DoneEast=1;
				_wall[i][0].bits.West=1;
				_wall[i][0].bits.DoneWest=1;
				_wall[MAZE_SIZE-1][i].bits.North=1;
				_wall[MAZE_SIZE-1][i].bits.DoneNorth=1;
				_wall[0][i].bits.South=1;
				_wall[0][i].bits.DoneSouth=1;
			}
		}
		void update(const MazePosition &pos, const MazeWall &wall){
			_wall[pos.y][pos.x] = wall;
			if(pos.x-1>=0){
				_wall[pos.y][pos.x-1].bits.East = wall.bits.West;
				_wall[pos.y][pos.x-1].bits.DoneEast = wall.bits.DoneWest;
			}
			if(pos.y-1>=0){
				_wall[pos.y-1][pos.x].bits.North = wall.bits.South;
				_wall[pos.y-1][pos.x].bits.DoneNorth = wall.bits.DoneSouth;
			}
			if(pos.x+1<MAZE_SIZE){
				_wall[pos.y][pos.x+1].bits.West = wall.bits.East;
				_wall[pos.y][pos.x+1].bits.DoneWest = wall.bits.DoneEast;
			}
			if(pos.y+1>=0){
				_wall[pos.y+1][pos.x].bits.South = wall.bits.North;
				_wall[pos.y+1][pos.x].bits.DoneSouth = wall.bits.DoneNorth;
			}
		}
		void update(const MazePosition &pos, bool left, bool flont, bool right, bool back = false){
			MazeWall wall;
			wall.update((pos.dir+0)&0x3, flont ? 1 : 0);
			wall.update((pos.dir+1)&0x3, right ? 1 : 0);
			wall.update((pos.dir+2)&0x3, back ? 1 : 0);
			wall.update((pos.dir+3)&0x3, left ? 1 : 0);
			wall |= 0xF0;
			update(pos, wall);
		}
		void printWall(uint8_t nums[MAZE_SIZE][MAZE_SIZE] = NULL){
			for(int y=MAZE_SIZE-1; y>=0; y--){
				for(int x=0; x<MAZE_SIZE; x++){
					printf("+%s+",_wall[y][x].bits.North?"---":"   ");
				}
				printf("\n");

				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s", _wall[y][x].bits.West?"|":" ");
					if(nums==NULL) printf("   ");
					else printf("%3d", nums[y][x]);
					printf("%s", _wall[y][x].bits.East?"|":" ");
				}
				printf("\n");

				for(int x=0; x<MAZE_SIZE; x++){
					printf("+%s+",_wall[y][x].bits.South?"---":"   ");
				}
				printf("\n");
			}
			printf("\n");
		}
		void loadFromArray(const char asciiData[MAZE_SIZE+1][MAZE_SIZE+1]){
			for(int i=0;i<MAZE_SIZE;i++){
				for(int j=0;j<MAZE_SIZE;j++){
					char ch = asciiData[MAZE_SIZE-1-i][j];
					if(('0' <= ch && ch <= '9') || ('a' <= ch && ch <= 'f')){
						uint8_t wall_bin;
						if('0' <= ch && ch <= '9') wall_bin = ch - '0';
						else wall_bin = ch - 'a' + 10;
						_wall[i][j].byte = wall_bin | 0xf0;
					}
				}
			}
		}
		uint8_t getWall(const MazePosition &pos){return _wall[pos.y][pos.x];}
		uint8_t getWall(const uint8_t x, const uint8_t y){return _wall[y][x];}
		uint8_t getStepMap(const MazePosition &pos){return stepMap[pos.y][pos.x];}
		uint8_t getStepMap(const uint8_t x, const uint8_t y){return stepMap[y][x];}
		void updateStepMap(const MazePosition &dist){
			for (int i=0;i<MAZE_SIZE;i++) {
				for (int j=0;j<MAZE_SIZE;j++) {
					stepMap[i][j] = 255;
				}
			}
			stepMap[dist.y][dist.x]=0;
			MazePosition cur = dist;
			bool changed = true;
			for(int i=0; changed; i++){
				changed = false;
				MazeWall wall = getWall(cur);
				for(int j=0; j<4; j++){
					if(!wall[j]){

					}
				}
			}
			printWall(stepMap);
		}
	private:
		MazeWall _wall[MAZE_SIZE][MAZE_SIZE];	// _wall[y][x]
		uint8_t stepMap[MAZE_SIZE][MAZE_SIZE];
};

class MazeAgent {
	public:
		MazeAgent(Maze *maze):maze(maze){
			state = IDLE;
		}
	private:
		Maze *maze;
		MazePosition current;
		MazePosition virtualDist;
		MazePosition destination;
		enum State{
			IDLE,
			SEARCHING_FOR_GOAL,
			SEARCHING_EXTRA,
			GOING_TO_START,
			FINISHED,
		}state;
};

int main(){
	Maze maze, maze_backup;
	MazeAgent agent(&maze);
	const char mazeData_55test[5+1][5+1] = {
		{"91513"},
		{"aad6a"},
		{"aad3a"},
		{"aafaa"},
		{"ec546"}
	};
	Maze maze_target;
	maze_target.loadFromArray(mazeData_55test);
	maze_target.printWall();
	maze_backup = maze;

	maze.updateStepMap(MazePosition(2,2,0));
	return 0;
}

