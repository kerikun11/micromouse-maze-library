#include <cstdio>
#include <cstdint>
#include "Maze.h"
#include "SearchAlgorithm.h"

#include <unistd.h>
#include <time.h>
#include <chrono>

using namespace MazeLib;

#define DISPLAY 0
#define MAZE_BACKUP_SIZE 5

const char mazeData_fp2016[8+1][8+1] = { "6beab6ab", "4aaa3c37", "c2ab4a1d", "b8a35683", "6a2954b5", "57575c29", "5549ca17", "dc8aaa9d", };

extern const char mazeData_maze[16+1][16+1] = {
	"9551553ff9551553",
	"af92ffc556ffaffa",
	"a96aff939553affa",
	"8452ffaaa9568552",
	"affc53aaaa95693a",
	"effff86c6c2ffaaa",
	"9395569553c15286",
	"aaafff813ad43aaf",
	"aaefffac68556aaf",
	"a85153c556d556c3",
	"ae96fabff93ffffa",
	"a96d7aaffac53ffa",
	"869556affaff8552",
	"abafffc556ffaffa",
	"aaad515153ffaffa",
	"eec55456fc554556",
};

extern const char mazeData_maze2013exp[16+1][16+1] = {
	"9795555555551393",
	"856915555553eaaa",
	"8796a95153d43c6a",
	"ad056ad07a93853a",
	"ad0796d07c6aad2a",
	"a943c3d0793ac3aa",
	"a8543ad056ac3aaa",
	"ac53ac38396baaaa",
	"a956a96c6c3c2aaa",
	"ac53c43939696aaa",
	"a95693c6c6bad2aa",
	"a8556a9153c296aa",
	"a8393c6c5296abaa",
	"aac681793c43a86a",
	"aabbec56c5546ad2",
	"ec44555555555456",
};

extern const char mazeData_maze2013fr[16+1][16+1] = {
	"9115151553ff9113",
	"aaafafaf94556aaa",
	"a8696fafa95556aa",
	"82fad543aa95556a",
	"aa92fffac6c55392",
	"a8681516f95556aa",
	"c2faafa954553faa",
	"f816afa83953afaa",
	"fac3856c6afaafaa",
	"92fac5553c3ac56a",
	"ac54539543ac5552",
	"affffaa93aaf9552",
	"8515542aac696952",
	"af851546c3fafafa",
	"afafaf9552fafafa",
	"efc5456ffc545456",
};

extern const char mazeData_maze3[16+1][16+1] = {
	"d5553fffffffffff",
	"d5116fff93ffffff",
	"ffe815556affffff",
	"fffeaf93fa93ffff",
	"ff95052afaaaffff",
	"ffc52baa96aaffff",
	"ff956c6c056c5553",
	"9507fff92ffffffa",
	"a96f955443fffffa",
	"aafbaffff8553ffa",
	"aef86ffffaffc156",
	"c53afffffafffaff",
	"b96a955552fffaff",
	"86beefbffafffaff",
	"8545156ffc5556fb",
	"efffeffffffffffe",
};

extern const char mazeData_maze4[16+1][16+1] = {
	"d51157f9515557d3",
	"97ac5552fc55153a",
	"afaff97ad153afaa",
	"c5413c52fad6c3c2",
	"fbfaabbc56f956fa",
	"d452ac053ffaf956",
	"d13aad6f8156d453",
	"faac2d392c39517a",
	"fc43afac47aefafa",
	"93bc43af9383fa96",
	"aac552c56c6a946b",
	"ac553c5555568552",
	"afffabffb9556fba",
	"affd04154695512a",
	"83938501552ffeea",
	"ec6c6feeffc55556",
};

extern const char mazeData_2017_East_MC[16+1][16+1] = {
	"6a2b63762b6aaa2b",
	"569695c1c3c23617",
	"5569696969695541",
	"5c9696968bd6941d",
	"5e29696962283543",
	"569696975dc35415",
	"55696969dea15541",
	"5c9696163ea15415",
	"5e2969d49ea15d41",
	"5696822837e09695",
	"5ca8a9569569e969",
	"c36a3e88a896a3c3",
	"e81e82aaaa356169",
	"6296b4a2a295d5c3",
	"5574282828282835",
	"dc89ca8a8a8a8bc9",
};

extern const char mazeData_MM2017CX[16+1][16+1] = {
	"762aaaaaaaaaa22b",
	"4956aa36aa363dc3",
	"4a1563c963c9ca35",
	"4a9c9caa9caa3695",
	"5636362362369c35",
	"55c9c9dc9dc9e295",
	"5c362b62a363e0b5",
	"5695c3543c9ca0b5",
	"5569695c9762a8b5",
	"5c968bcaa89caa35",
	"56a963763762b695",
	"5c369c09c0957c35",
	"569c37d63d694a95",
	"556a94a9ca975621",
	"49577caaaaa89dd5",
	"de888aaaaaaaaaa9",
};

extern const char mazeData_Cheese2017[16+1][16+1] = {
	"e2aaaaa377777777",
	"e0a2aab555555555",
	"e8202a3415555555",
	"7615435555555555",
	"49c8941555555555",
	"56363d4155555555",
	"55c9c3dd55555555",
	"5563697755555555",
	"5495c34155555555",
	"c961695555555555",
	"6a9dca9555555555",
	"5776363555555555",
	"c000955555555555",
	"7ddd694955555555",
	"4377c3d755555555",
	"dc88a8a89ddddddd",
};

const char mazeData_MM2012HX[32+1][32+1] = {
	"eaaaaaaaaaaaaaaaaaaaaaaaaaaaaaab",
	"6363636363636aaa363f6aa2a36aaaa3",
	"54141414141416a355ca8abca88aaa35",
	"55414141414155e8956aaaaa3636a295",
	"5554141414141caaa9563623555c3c35",
	"49c9c9c9c9ddcaaaa355540155569695",
	"56362222236aaaaaa9c95c095408bca1",
	"555400000156aaaaaaaa961695caaa35",
	"5554000001542aaaaaaa3555696aaa15",
	"55540000015556b6a23694155696aa95",
	"5554000001555d6969496955c9616aa1",
	"555c88888955569696969614a2155635",
	"55caaaaaaa9549696969695569555555",
	"5caa2aaaaaa896969696975556955555",
	"4363caaaaaaaa96968297555c8a9c9c1",
	"5554aaa36236a2968296881caaaaaa35",
	"5c9caa35554169682960235622236355",
	"4aaaaa9c9c9c968a9755555400015555",
	"436aaaaaaaaaa0222000809400015555",
	"5556aaaaaaaa3dddd55d683400015c95",
	"55556aaaaaa35622355682940001ca35",
	"555556aaaa355400155c283400016355",
	"5555556aa3555400155e829400015555",
	"555555563555540015563c3400015555",
	"55555555555554001555569c88895555",
	"555555c95555540015555caaaaaa9c95",
	"55555caa955554001555ca2a2a2aaa21",
	"5555caaaa9555400155ca34a0a82aa15",
	"555caaaaaa9554001556a14a1ea1ea15",
	"55caaaaaaaa95c889c94a9ca1ea1ea01",
	"54aaaaaaaaaa96aaaaa8aaaa8aa8a295",
	"dcaaaaaaaaaaa8aaaaaaaaaaaaaaa8a9",
};

const char mazeData_MM2013HX[32+1][32+1] = {
	"95555115555555395555555395555393",
	"a9153aa9515153aa9515153aa955382a",
	"aa816aac16bc16aac16bc16ac417aaaa",
	"a82816c16943c16c16943c3a9569442a",
	"aa86c396943c3c396945456c4514396a",
	"a8053c6947a96fc692fffffffd052c3a",
	"82852954556c5553aafffffffd05296a",
	"a8052a955539553aaafffffffd052c3a",
	"86c56aa9556c53aaaafffffffd056d2a",
	"c5553c6c555556aaaafffffffd0793aa",
	"d55385555515556aaafffffffd07ac6a",
	"913aafffffa95556aa9555555507c53a",
	"aaaaafffffaa95556ac53d515507956a",
	"aaaaafffffaaa9555295695055078552",
	"aaaaafffffaaaa9552c538545507853a",
	"aaaaafffffaaaa85545568551507afaa",
	"aaaaafffffaaaac5395554554547c56a",
	"aaaaafffffaaaa93aa95555555555552",
	"aac6afffffac6aac6aa955555555553a",
	"ac554555516d12affaaa9555555553aa",
	"8155155514796ac552aaaffffff93aaa",
	"a83943f9695454553aaaaffffffaaaaa",
	"82841696bc539553aaaaaffffffaaaaa",
	"ac4141456956a93aaaaaaffffffaaaaa",
	"853c16913c53aac46aaaaffffffaaaaa",
	"a94143802956ac5556aaaffffffaaaaa",
	"ac1416846c53855553aaaffffffaaaaa",
	"a94143839156c1553aaac5555556aaaa",
	"841416ac40553c156aac555555556aaa",
	"a941438554156d4152c55555555556aa",
	"805452c555455554545555555555556a",
	"ec555455555555555555555555555556",
};

const char mazeData_MM2014HX[32+1][32+1] = {
	"62aaaaaaaaaaaa2223636363636a2363",
	"5caaaaaaaaaaa35d5c14141414969555",
	"56aaaaaaaaaa35ca8341414149696955",
	"556362222223556a35dc9c1496968b55",
	"55554000000155569caaa3c96974aa15",
	"5555400000015555763635e28b556a95",
	"555540000001555549c9496962155635",
	"5555c8880001555c8a36968b40155555",
	"555caaa3400155caa349696340155541",
	"55caaa35c889556aa9d696954015c9d5",
	"54aaa35caaaa95ca3e296835401caa35",
	"556aa9caaaaaa96a968b4a95c89ea295",
	"55caaaaaaaaaaa9e2963563caaaaa0b5",
	"54aa2aa3636363769695c9caaaa3e0b5",
	"54a3ca3c9c9c9c1ca8a8a22aaaa1e0b5",
	"5c3ca356222223caaaaa355636a9e835",
	"434a3c140000016aaaaa9489c96aaa95",
	"55c3c3d400000142aaaaa8aaa3caa361",
	"557c283c8888015c2a2a3622356aa955",
	"5543c3c2aaa34156828294001556a355",
	"55403c3c3e35415ca8a8340015556955",
	"554003c3c3d5c956aaaa140015c9ca15",
	"5dc888bc3c3c2a956aaa940015eaaa95",
	"caaaaaa3c3c3c3f5caaa34009caaaa35",
	"6aaaaa35fc3c3ca16aa35c896236a295",
	"56aaa35563c3ca3d56354363401c2835",
	"556a35555c3ca3ca95555415c8168295",
	"55560955569634aaa89541416a082835",
	"5555ca955ca9c96aaa3c9c9c8a8a8a95",
	"555caaa9caaaaa96a3caaaaaaaaaaa35",
	"55caaaaaaaaaaaa97caaaaaaaaaaaa95",
	"dcaaaaaaaaaaaaaa8aaaaaaaaaaaaaa9",
};

const char mazeData_MM2015HX[32+1][32+1] = {
	"6aaaaaaaaa2aaaaaaaaaaaaaaaaaaaa3",
	"56aaaaaaa356aaaaaaa36aaa22aaaaa1",
	"556a22aa35556aaaaaa9caaa94aaaa35",
	"554a9caa81d5caaaaaa36aaa296aaa95",
	"55c36aaa3c356222223556aa83562235",
	"55695623c3c140000015556aa9540015",
	"554b54017c3d400000154956a3540015",
	"54969c8143c3c000001483c821540015",
	"55696a281c3c3c00001434aa95540015",
	"555e0a0a0a0bc3c000155562a95c8895",
	"5556828282837c3c8895555423caaa21",
	"5554a0a0a0a143c3e2a955c95436aa15",
	"555c282828295c3c3d7754aa15556355",
	"555e0a0a0a0bc3c3c3c1556b55c95c15",
	"555682828283683c3c3c89435caa9755",
	"555ca0a0a0a94a8bc3c3e2954aaaa095",
	"55caa8b43caa16aaa8bc28355e2aa961",
	"5caaaaa9ca2a156aaaaa8295c3caa355",
	"4a2b6aaaaa8a955622223c3c3caa3415",
	"5683562222236154000003c3c2a35555",
	"5c295400000155540000003c29695555",
	"568354000001555400000003c3429c15",
	"5c29540000015554000000003c1c3615",
	"568354000001555c888888888bc3d555",
	"5c295c88888955caaa2aaaaaaa3c3415",
	"568bcaaaaaaa9caaaa8aa2aaa3c3c155",
	"556aaaaa22aaaaaa236235623c3c3555",
	"5c96aaa3556aaaa35555555543ca1555",
	"56356a3555caaa35c95c148954aa1555",
	"4955c355556aa35563435c2a882a1555",
	"569ca9555556a9555555ca8aaa8a15c1",
	"dcaaaa9c9c9caa9c9c9caaaaaaaa88a9",
};
const char mazeData_MM2016HX[32+1][32+1] = {
	"76aaaaaaaaaaaaaa2b637762376236a3",
	"4836a36aaaaaaaa3c355401540154961",
	"4b55694b6aaaaaa83555554015409695",
	"4b55574bca36aaaa95554015401d6969",
	"4a955c0b6a9caaaa3554154015d69683",
	"56a1568bcaaaa2aa9555c89dc969e0a1",
	"55695ca36aaaa96a3c9d6222b69e2829",
	"555616a956aaaa835623c009696a0a0b",
	"5555556a9c2a36a941543c9697c28283",
	"555555caaa8a88a35c89c36961e0a0a1",
	"5555556236aaaa35caa2b49601682829",
	"5555c9401576aa9caa29683c014a0a0b",
	"555563c89c1caa2ab69697c3c9c28a8b",
	"55c9556363563e16a96961683ea8aaa3",
	"556355555555ca94b697554a16aaaaa9",
	"555c94955c9c2a29696155c29caaaa37",
	"554a34bc96aa969697555ca8a36363c1",
	"55d69d623562bd696895436a21555435",
	"5568b7c01494b6969634955695555555",
	"55c3683c9d696969695c3555695c1555",
	"556883c3e296969e1e8a895543435555",
	"55d63cbc3c296963563623555555c955",
	"55e1ca369683560149540155c1543695",
	"55e8a3c9697c9c89d69c895569c15c35",
	"5563e0b69e82aaaa2963635556355695",
	"c15569696a3ca363d69c15c1555c1c35",
	"689c969e174b7c9c2963d568954b5695",
	"562369e294968363d69ca9c2b5ca9c35",
	"5400963569696954356aaaa9696aaa95",
	"5c8969c9ca9e8a9dc9caaaaa9696aa35",
	"42ab42aaaaaaaaaaaaaaaaaaa8b57749",
	"dcaa9caaaaaaaaaaaaaaaaaaaaa8888b",
};

const char mazeData_MM2017HX[32+1][32+1] = {
	"63636aaa36236aaaaa36236aaaaa3623",
	"c955caa35c89caaaa35c89caaaa35c89",
	"6a956aa95e2a2aaa35caaaaaaaa9caa3",
	"c2b5caa343c3d6aa9caaaaa2aaa36361",
	"683562355c3c356236aaaa3562355c9d",
	"c295401543c35540156aaa954015caa3",
	"6835c8955c3c15c895caaa35c8957635",
	"c29c2ab5ca8a15eaa16aaa9caa2949c9",
	"e8368aa16aaa15eaa14aaa36a2834aab",
	"63556361562355637556235c28295623",
	"41541415540155555554015682835401",
	"c95541415c8955555554895c28295c89",
	"6a949c9d4aaa949c95c8aa8a8a8bcaa3",
	"c2b4aaa34aaabcaaa9e2a2a2aaa36a29",
	"e0b56235caaa376237e0a0b56235c283",
	"e0b540156aaa95401568283540156829",
	"e0b5c895caaa35c8954a0a15c8948283",
	"68b4aaa96aaa88aaa88a8a9caa296829",
	"ca34aaabcaaa362a222aaa3636174a8b",
	"635caaa3762355d7d556234141415623",
	"4156aaa9540154282154015414155401",
	"c95caaa35c8955d7d55c894141415c89",
	"6a96aa294aaa9c28a9caa2949c9c8aa3",
	"ca3caa835636368aa36a2834aaa22221",
	"635762354141416235c2829562355555",
	"55554015541415401568283540155555",
	"5c95c895414155c894828295c8955555",
	"42b4aaa1dc1c9caa29e8a8a8aaa9c881",
	"5c3caa356a8a3636176aaa22a2a37775",
	"5756aa95562355414156235561600001",
	"555caa35540154141554015494955dd5",
	"ddcaaa9c9c89c9c9c89c89dca8a88aa9",
};

#if MAZE_SIZE == 8
std::vector<Vector> goal = {Vector(7,7)};
Maze sample(mazeData_fp2016);
#elif MAZE_SIZE == 16
std::vector<Vector> goal = {Vector(7,7),Vector(7,8),Vector(8,8),Vector(8,7)};
// std::vector<Vector> goal = {Vector(3,3),Vector(3,4),Vector(4,3),Vector(4,4)};
//Maze sample(mazeData_maze, false);
//Maze sample(mazeData_maze3, false);
//Maze sample(mazeData_maze4, false);
//Maze sample(mazeData_maze2013fr, false);
//Maze sample(mazeData_maze2013exp, false);
// Maze sample(mazeData_2017_East_MC, true);
Maze sample(mazeData_MM2017CX, true);
// Maze sample(mazeData_Cheese2017, true);
#elif MAZE_SIZE == 32
#define YEAR 2014
#if YEAR == 2012
std::vector<Vector> goal = {Vector(22,25)};
Maze sample(mazeData_MM2012HX);
#elif YEAR == 2013
std::vector<Vector> goal = {Vector(6,5), Vector(6,6), Vector(6,7), Vector(7,5), Vector(7,6), Vector(7,7), Vector(8,5), Vector(8,6), Vector(8,7)};
Maze sample(mazeData_MM2013HX, false);
#elif YEAR == 2014
std::vector<Vector> goal = {Vector(26,5)};
Maze sample(mazeData_MM2014HX);
#elif YEAR == 2015
std::vector<Vector> goal = {Vector(7,24)};
Maze sample(mazeData_MM2015HX);
#elif YEAR ==2016
std::vector<Vector> goal = {Vector(1,2), Vector(1,3), Vector(1,4), Vector(2,2), Vector(2,3), Vector(2,4), Vector(3,2), Vector(3,3), Vector(3,4)};
// std::vector<Vector> goal = {Vector(0,31)};
Maze sample(mazeData_MM2016HX);
#elif YEAR ==2017
std::vector<Vector> goal = {Vector(19,20)};
Maze sample(mazeData_MM2017HX);
#endif
#endif

#if 1

Maze maze;
SearchAlgorithm searchAlgorithm(maze, goal);
auto max_usec = 0;
auto start = std::chrono::system_clock::now();
auto end = std::chrono::system_clock::now();       // 計測終了時刻を保存
auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
int step=0,f=0,l=0,r=0,b=0,k=0; /**< 探索の評価のためのカウンタ */
int wall_log=0,log_max=0;

void queueActions(const std::vector<Dir>& nextDirs){
	#if DISPLAY
	// usleep(200000);
	#endif
	for(const auto& nextDir: nextDirs){
		#if DISPLAY
		// char c; scanf("%c", &c);
		searchAlgorithm.printInfo();
		printf("Step: %4d, Forward: %3d, Left: %3d, Right: %3d, Back: %3d, Known: %3d\n", step, f, l, r, b, k);
		printf("It took %5d [us], the max is %5d [us]\n", usec, max_usec);
		printf("wall_log: %5d, log_max: %5d\n", wall_log, log_max);
		usleep(100000);
		#endif
		auto nextVec = searchAlgorithm.getCurVec().next(nextDir);
		switch (Dir(nextDir - searchAlgorithm.getCurDir())) {
			case Dir::Forward:
			/* queue SearchRun::GO_STRAIGHT */
			f++;
			break;
			case Dir::Left:
			/* queue SearchRun::TURN_LEFT_90 */
			l++;
			break;
			case Dir::Right:
			/* queeu SearchRun::TURN_RIGHT_90 */
			r++;
			break;
			case Dir::Back:
			/* queue SearchRun::TURN_BACK */
			b++;
			wall_log=0;
			break;
		}
		searchAlgorithm.updateCurVecDir(nextVec, nextDir);
		step++;
	}
}

void stopAndSaveMaze(){
	/* queue Action::STOP */
	/* wait for queue being empty */
	/* stop the robot */
	/* backup maze to flash memory */
	const auto& v = searchAlgorithm.getCurVec();
	const auto& d = searchAlgorithm.getCurDir();
	searchAlgorithm.updateCurVecDir(v.next(d + 2), d + 2); // u-turn
	/* queue Action::RETURN */
	/* queue Action::GO_HALF */
	/* start the robot */
}

bool searchRun(const bool isStartStep = true, const Vector& startVec = Vector(0, 0), const Dir& startDir = Dir::North){
	searchAlgorithm.reset();
	searchAlgorithm.updateCurVecDir(startVec, startDir);
	searchAlgorithm.calcNextDir();
	if(searchAlgorithm.getState() == SearchAlgorithm::REACHED_START) return true;
	if(isStartStep) {
		/* queue Action::START_STEP */
		searchAlgorithm.updateCurVecDir(startVec.next(startDir), startDir);
	}
	/* conduct calibration of sensors */
	/* start the robot */
	int count=0;
	while(1){
		// if(count++>50) return false; // for debug
		const auto& v = searchAlgorithm.getCurVec();
		const auto& d = searchAlgorithm.getCurDir();
		SearchAlgorithm::State prevState = searchAlgorithm.getState();
		start = std::chrono::system_clock::now();
		searchAlgorithm.calcNextDir(); //< 時間がかかる処理！
		end = std::chrono::system_clock::now();
		usec = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
		if(max_usec < usec) max_usec = usec;
		SearchAlgorithm::State newState = searchAlgorithm.getState();
		if(newState != prevState && newState == SearchAlgorithm::REACHED_GOAL){
		}
		if(newState != prevState && newState == SearchAlgorithm::SEARCHING_ADDITIONALLY){
			stopAndSaveMaze();
			continue;
		}
		if(newState != prevState && newState == SearchAlgorithm::BACKING_TO_START){
			stopAndSaveMaze();
			continue;
		}
		if(newState != prevState && newState == SearchAlgorithm::GOT_LOST){
			/* queue SearchRun::STOP */
			/* wait for queue being empty */
			/* stop the robot */
			printf("\n");
			printf("Got Lost!");
			while (1);
			return false;
		}

		// 既知区間移動をキューにつめる
		queueActions(searchAlgorithm.getNextDirs());
		k += searchAlgorithm.getNextDirs().size();

		// reached start and searching finised
		if(v == Vector(0, 0)) break;

		/* wait for queue being empty */

		// find walls
		if(!maze.isKnown(v, d+1)) wall_log++;
		if(!maze.isKnown(v, d+0)) wall_log++;
		if(!maze.isKnown(v, d-1)) wall_log++;
		searchAlgorithm.updateWall(v, d+1, sample.isWall(v, d+1)); // left wall
		searchAlgorithm.updateWall(v, d+0, sample.isWall(v, d+0)); // front wall
		searchAlgorithm.updateWall(v, d-1, sample.isWall(v, d-1)); // right wall
		if(log_max < wall_log) log_max = wall_log;
		// if(!maze.isWall(v,d)) searchAlgorithm.updateWall(v.next(d), d, sample.isWall(v.next(d), d)); // front wall
		/* backup the wall */

		// 候補の中で行ける方向を探す
		const auto nextDirsInAdvance = searchAlgorithm.getNextDirsInAdvance();
		const auto nextDirInAdvance = *std::find_if(nextDirsInAdvance.begin(), nextDirsInAdvance.end(), [&](const Dir& dir){
			return !maze.isWall(v, dir);
		});
		queueActions({nextDirInAdvance});
	}
	/* queue Action::START_INIT */
	searchAlgorithm.updateCurVecDir(Vector(0, 0), Dir::North);
	searchAlgorithm.calcNextDir(); //< 時間がかかる処理！
	/* wait for queue being empty */
	/* stop the robot */
	/* backup the maze */
	// 最短経路が導出できるか確かめる
	if (!searchAlgorithm.calcShortestDirs()) {
		printf("Couldn't solve the maze!\n");
		return false;
	}
	return true;
}

bool fastRun(){
	if(!searchAlgorithm.calcShortestDirs()){
		printf("Failed to find shortest path!\n");
		return false;
	}
	/* move robot here */
	return true;
}

#endif

int main(void){
	setvbuf(stdout, (char *)NULL, _IONBF, 0);
	#if 1
	while(!searchRun());
	searchAlgorithm.printInfo();
	printf("the max is %5d [us]\n", max_usec);
	printf("the log_max is %5d\n", log_max);
	fastRun();
	searchAlgorithm.printPath();
	searchAlgorithm.calcShortestDirs(false);
	searchAlgorithm.printPath();
	#else
	// Maze sample2(mazeData_maze2013half, false);
	// printf("diff: %d\n", maze.diffKnownWall(sample2));
	StepMap stepMap(maze);
	stepMap.update({Vector(0,31)});
	stepMap.print();
	#endif
	return 0;
}
