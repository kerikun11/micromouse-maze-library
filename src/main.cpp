#include <cstdio>
#include <cstdint>
#include "Maze.h"

#define DISPLAY 0

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

const char mazeData_maze2016half[32+1][32+1] = {
	{"76aaaaaaaaaaaaaa2b637762376236a3"},
	{"4836a36aaaaaaaa3c355401540154961"},
	{"4b55694b6aaaaaa83555554015409695"},
	{"4b55574bca36aaaa95554015401d6969"},
	{"4a955c0b6a9caaaa3554154015d69683"},
	{"56a1568bcaaaa2aa9555c89dc969e0a1"},
	{"55695ca36aaaa96a3c9d6222b69e2829"},
	{"555616a956aaaa835623c009696a0a0b"},
	{"5555556a9c2a36a941543c9697c28283"},
	{"555555caaa8a88a35c89c36961e0a0a1"},
	{"5555556236aaaa35caa2b49601682829"},
	{"5555c9401576aa9caa29683c014a0a0b"},
	{"555563c89c1caa2ab69697c3c9c28a8b"},
	{"55c9556363563e16a96961683ea8aaa3"},
	{"556355555555ca94b697554a16aaaaa9"},
	{"555c94955c9c2a29696155c29caaaa37"},
	{"554a34bc96aa969697555ca8a36363c1"},
	{"55d69d623562bd696895436a21555435"},
	{"5568b7c01494b6969634955695555555"},
	{"55c3683c9d696969695c3555695c1555"},
	{"556883c3e296969e1e8a895543435555"},
	{"55d63cbc3c296963563623555555c955"},
	{"55e1ca369683560149540155c1543695"},
	{"55e8a3c9697c9c89d69c895569c15c35"},
	{"5563e0b69e82aaaa2963635556355695"},
	{"c15569696a3ca363d69c15c1555c1c35"},
	{"689c969e174b7c9c2963d568954b5695"},
	{"562369e294968363d69ca9c2b5ca9c35"},
	{"5400963569696954356aaaa9696aaa95"},
	{"5c8969c9ca9e8a9dc9caaaaa9696aa35"},
	{"42ab42aaaaaaaaaaaaaaaaaaa8b57749"},
	{"dcaa9caaaaaaaaaaaaaaaaaaaaa8888b"},
};

int main(void){
	setvbuf(stdout, (char *)NULL, _IONBF, 0);

#if MAZE_SIZE == 8
	std::vector<Vector> goal = {Vector(7,7)};
	Maze sample(goal, mazeData_fp2016);
#elif MAZE_SIZE == 16
	std::vector<Vector> goal = {Vector(7,7),Vector(7,8),Vector(8,8),Vector(8,7)};
	Maze sample(goal, mazeData_maze, false);
	//Maze sample(goal, mazeData_maze2013exp, false);
#elif MAZE_SIZE == 32
#if 0
	std::vector<Vector> goal = {Vector(7,6)};
	Maze sample(goal, mazeData_maze2013half, false);
#else
	std::vector<Vector> goal = {Vector(3,3)};
	Maze sample(goal, mazeData_maze2016half);
#endif
#endif

	Maze maze(goal), backup(goal);
	Agent agent(maze);
	while(1){
		agent.calcNextDir();
		if(agent.getState() == Agent::REACHED_START) break;
		if(agent.getState() == Agent::GOT_LOST){
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
			agent.updateCurVecDir(nextVec, nextDir);
		}
		const Vector& v = agent.getCurVec();
		const Dir& d = agent.getCurDir();
		agent.updateWall(v, d-1, sample.isWall(v, d-1));
		agent.updateWall(v, d+0, sample.isWall(v, d+0));
		agent.updateWall(v, d+1, sample.isWall(v, d+1));
#if DISPLAY
		usleep(400000);
#endif
	}
	agent.printInfo();
	sleep(1);
	if(!agent.calcShortestPath()){
		printf("Failed to find shortest path!\n");
	}else{
		agent.printPath();
	}
	return 0;
}

