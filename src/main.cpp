#include <cstdio>
#include <queue>

#define MAZE_SIZE	5

#define NORTH		0x01	// North Wall
#define EAST		0x02
#define SOUTH		0x04
#define WEST		0x08
#define D_NORTH		0x10	// Done North Wall
#define D_EAST		0x20
#define D_SOUTH		0x40
#define D_WEST		0x80
#define F_NORTH		0x11	// Found North Wall
#define F_EAST		0x22
#define F_SOUTH		0x44
#define F_WEST		0x88
#define N_NORTH		0x10	// Not Found North Wall
#define N_EAST		0x20
#define N_SOUTH		0x40
#define N_WEST		0x80

struct IndexVec{
	public:
		int8_t x;
		int8_t y;

		IndexVec(int8_t x=0, int8_t y=0):x(x), y(y) {}
		inline IndexVec operator+(const IndexVec &obj) const { return IndexVec(x+obj.x, y+obj.y); }
		inline IndexVec operator-(const IndexVec &obj) const { return IndexVec(x-obj.x, y-obj.y); }
		inline void operator+=(const IndexVec &obj) { x+=obj.x; y+=obj.y; }
		inline void operator-=(const IndexVec &obj) { x-=obj.x; y-=obj.y; }
		inline const IndexVec& operator=(const IndexVec &obj) { x=obj.x; y=obj.y; return *this; }
		inline bool operator==(const IndexVec &obj) const { return x == obj.x && y == obj.y; }
		inline bool operator!=(const IndexVec &obj) const { return x != obj.x || y != obj.y; }
};

class Maze{
	public:
		Maze(){
			reset();
		}
		void reset(){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					wall[y][x]=0x00;
			for(int i=0; i<MAZE_SIZE; i++){
				wall[i][MAZE_SIZE-1] |= EAST|D_EAST;
				wall[i][0] |= WEST|D_WEST;
				wall[MAZE_SIZE-1][i] |= NORTH|D_NORTH;
				wall[0][i] |= SOUTH|D_SOUTH;
			}
		}
		inline Maze& operator=(const Maze& obj){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					wall[y][x] = obj.wall[y][x];
					stepMap[y][x] = obj.stepMap[y][x];
				}
			return *this;
		}
		void loadFromArray(const char asciiData[MAZE_SIZE+1][MAZE_SIZE+1]){
			for(int i=0;i<MAZE_SIZE;i++){
				for(int j=0;j<MAZE_SIZE;j++){
					char ch = asciiData[MAZE_SIZE-1-i][j];
					if(('0' <= ch && ch <= '9') || ('a' <= ch && ch <= 'f')){
						uint8_t w;
						if('0' <= ch && ch <= '9') w = ch - '0';
						else w = ch - 'a' + 10;
						wall[i][j] = w|0xf0;
					}
				}
			}
		}
		inline uint8_t getWall(const uint8_t x, const uint8_t y) const {return wall[y][x];}
		inline uint8_t getStepMap(const uint8_t x, const uint8_t y) const {return stepMap[y][x];}
		void updateWall(const uint8_t x, const uint8_t y, const uint8_t w){
			wall[y][x] |= w;
			if(x != 0          ) wall[y][x-1] |= (w&F_WEST )>>2; //< Not Westernmost
			if(x != MAZE_SIZE-1) wall[y][x+1] |= (w&F_EAST )<<2; //< Not Easternmost
			if(y != 0          ) wall[y-1][x] |= (w&F_SOUTH)>>2; //< Not Southernmost
			if(y != MAZE_SIZE-1) wall[y+1][x] |= (w&F_NORTH)<<2; //< Not Northernmost
		}
		inline void updateWall(const IndexVec& v, const uint8_t w) { return updateWall(v.x, v.y, w);}
		void updateWall(const uint8_t x, const uint8_t y, const uint8_t dir, const bool left, const bool front, const bool right){
			uint8_t w=0x00;
			w |= (left  ? F_NORTH : N_NORTH) << ((dir+3)&0x03);
			w |= (front ? F_NORTH : N_NORTH) << ((dir+0)&0x03);
			w |= (right ? F_NORTH : N_NORTH) << ((dir+1)&0x03);
			updateWall(x,y,w);
		}
		void printWall(const IndexVec& v){
			for(int16_t y=MAZE_SIZE-1; y>=0; y--){
				for(uint8_t x=0; x<MAZE_SIZE; x++) printf("+%s+", wall[y][x]&NORTH ? "---" : "   ");
				printf("\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s", wall[y][x]&WEST ? "|" : " ");
					if(x==v.x&&y==v.y) printf(" * ");
					else printf("   ");
					printf("%s", wall[y][x]&EAST ? "|" : " ");
				}
				printf("\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++) printf("+%s+", wall[y][x]&SOUTH ? "---" : "   ");
				printf("\n");
			}
			printf("\n");
		}
		void printWall(uint8_t nums[MAZE_SIZE][MAZE_SIZE] = NULL){
			for(int16_t y=MAZE_SIZE-1; y>=0; y--){
				for(uint8_t x=0; x<MAZE_SIZE; x++) printf("+%s+", wall[y][x]&NORTH ? "---" : "   ");
				printf("\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					printf("%s", wall[y][x]&WEST ? "|" : " ");
					if(nums==NULL) printf("   ");
					else printf("%3d", nums[y][x]);
					printf("%s", wall[y][x]&EAST ? "|" : " ");
				}
				printf("\n");
				for(uint8_t x=0; x<MAZE_SIZE; x++) printf("+%s+", wall[y][x]&SOUTH ? "---" : "   ");
				printf("\n");
			}
			printf("\n");
		}
		void updateStepMap(const IndexVec& dest){
			for(uint8_t y=0; y<MAZE_SIZE; y++)
				for(uint8_t x=0; x<MAZE_SIZE; x++)
					stepMap[y][x] = 0xff;
			stepMap[dest.y][dest.x] = 0;
			std::queue<IndexVec> q;
			q.push(dest);
			while(q.size()){
				IndexVec focus = q.front(); q.pop();
				uint8_t focus_step = stepMap[focus.y][focus.x];
				uint8_t focus_wall = wall[focus.y][focus.x];
				if(!(focus_wall&NORTH) && stepMap[focus.y+1][focus.x]>focus_step+1){
					stepMap[focus.y+1][focus.x] = focus_step+1;
					q.push(IndexVec(focus.x, focus.y+1));
				}
				if(!(focus_wall&SOUTH) && stepMap[focus.y-1][focus.x]>focus_step+1){
					stepMap[focus.y-1][focus.x] = focus_step+1;
					q.push(IndexVec(focus.x, focus.y-1));
				}
				if(!(focus_wall&EAST) && stepMap[focus.y][focus.x+1]>focus_step+1){
					stepMap[focus.y][focus.x+1] = focus_step+1;
					q.push(IndexVec(focus.x+1, focus.y));
				}
				if(!(focus_wall&WEST) && stepMap[focus.y][focus.x-1]>focus_step+1){
					stepMap[focus.y][focus.x-1] = focus_step+1;
					q.push(IndexVec(focus.x-1, focus.y));
				}
			}
			printWall(stepMap);
		}
	private:
		uint8_t wall[MAZE_SIZE][MAZE_SIZE];
		uint8_t stepMap[MAZE_SIZE][MAZE_SIZE];
};

class MazeAgent{
	public:
		MazeAgent(Maze* maze, const IndexVec& dest):maze(maze), dest(dest){
			state = SEARCHING_FOR_GOAL;
		}
		enum State{
			SEARCHING_FOR_GOAL,
			SEARCHING_EXTRA,
			GOING_TO_START,
			FINISHED,
		};
		enum Action{
			GO_STRAIGHT,
			TURN_LEFT_90,
			TURN_RIGHT_90,
			RETURN,
		};
		enum State getState(){
			return state;
		}
		IndexVec getNextVector(const IndexVec& cur, uint8_t cur_wall){
			if(state == SEARCHING_FOR_GOAL){
				maze->updateWall(cur, cur_wall);
				maze->updateStepMap(dest);
				if(cur == dest){
					printf("Finished!\n");
					state = FINISHED;
				}else{
					uint8_t cur_step = maze->getStepMap(cur.x, cur.y);
					if(!(cur_wall&NORTH) && maze->getStepMap(cur.x, cur.y+1)==cur_step-1){
						return cur+IndexVec(0, 1);
					}
					if(!(cur_wall&SOUTH) && maze->getStepMap(cur.x, cur.y-1)==cur_step-1){
						return cur+IndexVec(0, -1);
					}
					if(!(cur_wall&EAST) && maze->getStepMap(cur.x+1, cur.y)==cur_step-1){
						return cur+IndexVec(1, 0);
					}
					if(!(cur_wall&WEST) && maze->getStepMap(cur.x-1, cur.y)==cur_step-1){
						return cur+IndexVec(-1, 0);
					}
				}
			}
		}
	private:
		Maze *maze;
		enum State state;
		IndexVec dest;
};

int main(){
	Maze maze, maze_backup;
	IndexVec dest(2,2);
	MazeAgent agent(&maze, dest);

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

	IndexVec cur;
	while(agent.getState() != MazeAgent::FINISHED){
		IndexVec next = agent.getNextVector(cur, maze_target.getWall(cur.x, cur.y));
		cur = next;
		maze.printWall(cur);
	}
	/*
	   maze.updateWall(0,0,0,true,false,true);
	   maze.updateWall(0,1,0,true,true,false);
	   maze.updateWall(1,1,1,true,false,true);
	   maze.updateWall(2,1,1,true,true,false);
	   maze.updateWall(2,0,2,false,true,false);
	   maze.updateWall(3,0,1,false,true,true);
	   maze.updateWall(3,1,0,true,false,true);
	   maze.updateWall(3,2,0,true,false,true);
	   maze.updateWall(3,3,0,true,true,false);
	   maze.updateWall(4,3,1,false,true,false);
	   maze.updateWall(4,4,0,false,true,true);
	   */

	return 0;
}
