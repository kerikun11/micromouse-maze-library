#include <cstdio>
#include <queue>

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
		inline int nWall() const {
			uint8_t b = byte & 0x0f;
			b = (b & 0x55) + (b >> 1 & 0x55);
			b = (b & 0x33) + (b >> 2 & 0x33);
			b = (b & 0x0f) + (b >> 4 & 0x0f);
			return b;
		}
		inline int nDoneWall() const {
			uint8_t b = (byte & 0xf0) >> 4;
			b = (b & 0x55) + (b >> 1 & 0x55);
			b = (b & 0x33) + (b >> 2 & 0x33);
			b = (b & 0x0f) + (b >> 4 & 0x0f);
			return b;
		}
};

class MazeVector {
	public:
		MazeVector(int8_t x=0, int8_t y=0) : x(x), y(y) {}
		int8_t x;
		int8_t y;
		inline MazeVector operator+(const MazeVector &obj) const { return MazeVector(x+obj.x, y+obj.y); }
		inline MazeVector operator-(const MazeVector &obj) const { return MazeVector(x-obj.x, y-obj.y); }
		inline void operator+=(const MazeVector &obj) { x+=obj.x; y+=obj.y; }
		inline void operator-=(const MazeVector &obj) { x-=obj.x; y-=obj.y; }
		inline const MazeVector& operator=(const MazeVector &obj) { x=obj.x; y=obj.y; return *this; }
		inline bool operator==(const MazeVector &obj) const { return x == obj.x && y == obj.y; }
		inline bool operator!=(const MazeVector &obj) const { return x != obj.x || y != obj.y; }
		static const MazeVector dirVector(int dir){
			switch(dir){
				case 0:
					return MazeVector(0,1);
				case 1:
					return MazeVector(1,0);
				case 2:
					return MazeVector(0,-1);
				case 3:
					return MazeVector(-1,0);
				default:
					return MazeVector();
			}
		}
};

class Maze {
	public:
		Maze(){
			reset();
		}
		const Maze& operator=(const Maze &obj){
			for (int i=0;i<MAZE_SIZE;i++) {
				for (int j=0;j<MAZE_SIZE;j++) {
					wall[i][j] = obj.wall[i][j];
					stepMap[i][j] = obj.stepMap[i][j];
				}
			}
			return *this;
		}
		void reset(){
			for(int i=0; i<MAZE_SIZE; i++){
				wall[i][MAZE_SIZE-1].bits.East=1;
				wall[i][MAZE_SIZE-1].bits.DoneEast=1;
				wall[i][0].bits.West=1;
				wall[i][0].bits.DoneWest=1;
				wall[MAZE_SIZE-1][i].bits.North=1;
				wall[MAZE_SIZE-1][i].bits.DoneNorth=1;
				wall[0][i].bits.South=1;
				wall[0][i].bits.DoneSouth=1;
			}
		}
		void update(const MazeVector &vec, const MazeWall &w){
			wall[vec.y][vec.x] = w;
			if(vec.x-1>=0){
				wall[vec.y][vec.x-1].bits.East = w.bits.West;
				wall[vec.y][vec.x-1].bits.DoneEast = w.bits.DoneWest;
			}
			if(vec.y-1>=0){
				wall[vec.y-1][vec.x].bits.North = w.bits.South;
				wall[vec.y-1][vec.x].bits.DoneNorth = w.bits.DoneSouth;
			}
			if(vec.x+1<MAZE_SIZE){
				wall[vec.y][vec.x+1].bits.West = w.bits.East;
				wall[vec.y][vec.x+1].bits.DoneWest = w.bits.DoneEast;
			}
			if(vec.y+1<MAZE_SIZE){
				wall[vec.y+1][vec.x].bits.South = w.bits.North;
				wall[vec.y+1][vec.x].bits.DoneSouth = w.bits.DoneNorth;
			}
		}
		void update(const MazeVector &vec, bool left, bool front, bool right, bool back = false){
			/*
			   MazeWall w;
			   w.update((vec.dir+0)&0x3, front ? 1 : 0);
			   w.update((vec.dir+1)&0x3, right ? 1 : 0);
			   w.update((vec.dir+2)&0x3, back ? 1 : 0);
			   w.update((vec.dir+3)&0x3, left ? 1 : 0);
			   w |= 0xF0;
			   update(vec, w);
			   */
		}
		void printWall(uint8_t nums[MAZE_SIZE][MAZE_SIZE] = NULL){
			for(int y=MAZE_SIZE-1; y>=0; y--){
				for(int x=0; x<MAZE_SIZE; x++){
					printf("+%s+",wall[y][x].bits.North?"---":"   ");
				}
				printf("\n");

				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s", wall[y][x].bits.West?"|":" ");
					if(nums==NULL) printf("   ");
					else printf("%3d", nums[y][x]);
					printf("%s", wall[y][x].bits.East?"|":" ");
				}
				printf("\n");

				for(int x=0; x<MAZE_SIZE; x++){
					printf("+%s+",wall[y][x].bits.South?"---":"   ");
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
						uint8_t w;
						if('0' <= ch && ch <= '9') w = ch - '0';
						else w = ch - 'a' + 10;
						wall[i][j].byte = w | 0xf0;
					}
				}
			}
		}
		uint8_t getWall(const MazeVector &vec){return wall[vec.y][vec.x];}
		uint8_t getWall(const uint8_t x, const uint8_t y){return wall[y][x];}
		uint8_t getStepMap(const MazeVector &vec){return stepMap[vec.y][vec.x];}
		uint8_t getStepMap(const uint8_t x, const uint8_t y){return stepMap[y][x];}
		void updateStepMap(const MazeVector &dist, bool onlyUseFoundWall = false){
			for (int i=0;i<MAZE_SIZE;i++) {
				for (int j=0;j<MAZE_SIZE;j++) {
					stepMap[i][j] = 255;
				}
			}
			stepMap[dist.y][dist.x]=0;
			std::queue<MazeVector> q;
			q.push(dist);
			while(q.size()){
				const MazeVector cur = q.front();
				q.pop();
				MazeWall curWall = wall[cur.y][cur.x];
				for(int i=0; i<4; i++){
					const MazeVector scan = cur + MazeVector::dirVector(i);
					const uint8_t curStep = stepMap[cur.y][cur.x];
					if (!curWall[i] && stepMap[scan.y][scan.x] > curStep +1) {
						if (onlyUseFoundWall && !curWall[i+4]) continue;
						stepMap[scan.y][scan.x] = curStep +1;
						if (wall[scan.y][scan.x].nWall() != 3) {
							q.push(scan);
						}
					}
				}
			}
			printWall(stepMap);
		}
	private:
		MazeWall wall[MAZE_SIZE][MAZE_SIZE];	// wall[y][x]
		uint8_t stepMap[MAZE_SIZE][MAZE_SIZE];
};

class MazeAgent {
	public:
		MazeAgent(Maze *maze):maze(maze){
			state = IDLE;
		}
	private:
		Maze *maze;
		MazeVector current;
		MazeVector virtualDist;
		MazeVector destination;
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

	maze.updateStepMap(MazeVector(2,2));
	return 0;
}

