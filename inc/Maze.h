/**
* @file Maze.h
* @brief マイクロマウスの迷路クラスを定義
* @author KERI (Github: kerikun11)
* @url https://kerikeri.top/
* @date 2017.10.30
*/
#pragma once

// inc
#include <vector>
#include <array>
#include <iostream>
#include <fstream>

// src
#include <algorithm>
#include <queue>
#include <iomanip> //< for std::setw()

namespace MazeLib {
	/** @def MAZE_SIZE
	*   @brief 迷路の1辺の区画数
	*/
	#define MAZE_SIZE			32
	/** @typedef
	*   @brief 迷路のサイズのbit数の整数型
	*   32x32の迷路ならuint32_t，16x16ならuint16_t，8x8ならuint8_t
	*/
	typedef uint32_t wall_size_t;

	/** @def
	*   @brief 迷路のカラー表示切替
	*/
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

	/** @struct Dir
	*   @brief 迷路上の方向を定義
	*/
	struct Dir {
	public:
		/** @enum Dir::AbsoluteDir
		*   @brief 絶対方向の列挙型
		*/
		enum AbsoluteDir: int8_t { East, North, West, South, AbsMax };
		/** @enum Dir::RelativeDir
		*   @brief 相対方向の列挙型
		*/
		enum RelativeDir: int8_t { Front, Left, Back, Right, RelMax };
		/** @function Constructor
		*   @param d Absolute Direction
		*/
		Dir(const enum AbsoluteDir d = East) : d(d) {}
		Dir(const int8_t d) : d(AbsoluteDir(d&3)) {}

		/** @brief 整数へのキャスト
		*/
		operator int8_t() const { return d; }
		/** @brief 代入演算子のオーバーロード
		*/
		const Dir operator=(const Dir& obj) { d=obj.d; return *this; }

		/** @function ordered
		*   @brief 「正面，左，右，後」の順序の方向配列を生成する関数
		*/
		// const std::array<Dir, 4> ordered(const Dir& prev) const
		// {
		// 	switch (Dir(d-prev)) {
		// 		case Front: return std::array<Dir, 4>{d, d+1, d-1, d+2};
		// 		case Left:  return std::array<Dir, 4>{d-1, d, d+1, d+2};
		// 		case Right: return std::array<Dir, 4>{d+1, d, d-1, d+2};
		// 		case Back:  return std::array<Dir, 4>{d, d+1, d-1, d+2};
		// 		default:    return std::array<Dir, 4>{d, d+1, d-1, d+2};
		// 	}
		// }
		/** @function All
		*   @brief 全方向の方向配列を生成する静的関数
		*/
		static const std::array<Dir, 4>& All()
		{
			static const std::array<Dir, 4> all = {East, North, West, South};
			return all;
		}
		/** @function <<
		*/
		friend std::ostream& operator<<(std::ostream& os, const Dir& d)
		{
			return os << (char)(">^<v "[d]);
		}
	private:
		enum AbsoluteDir d; /**< @brief 方向の実体 */
	};
	/** @typedef Dirs
	*   @brief Dir構造体の動的配列
	*/
	typedef std::vector<Dir> Dirs;

	/** @struct Vector
	*   @brief 迷路上の座標を定義．左下の区画が (0,0) の (x,y) 平面
	*/
	union Vector {
	public:
		struct{
			int8_t x; /**< @brief 迷路の区画座標 */
			int8_t y; /**< @brief 迷路の区画座標 */
		};
		uint16_t all; //< まとめて扱うとき用 */
		Vector(int8_t x=0, int8_t y=0) : x(x), y(y) {} /**< @brief コンストラクタ */
		Vector(const Vector& obj) : all(obj.all) {} /**< @brief コンストラクタ */
		/** @brief 演算子のオーバーロード
		*/
		const Vector operator+(const Vector& obj) const { return Vector(x+obj.x, y+obj.y); }
		const Vector operator-(const Vector& obj) const { return Vector(x-obj.x, y-obj.y); }
		const Vector& operator=(const Vector& obj) { all=obj.all; return *this; }
		bool operator==(const Vector& obj) const { return all==obj.all; }
		bool operator!=(const Vector& obj) const { return all!=obj.all; }
		/** @function next
		*   @brief 自分の引数方向に隣接した区画のVectorを返す
		*   @param 隣接方向
		*   @return 隣接座標
		*/
		const Vector next(const Dir &dir) const
		{
			switch(dir){
				case Dir::East: return Vector(x+1, y);
				case Dir::North: return Vector(x, y+1);
				case Dir::West: return Vector(x-1, y);
				case Dir::South: return Vector(x, y-1);
			}
			printf("Warning: invalid direction\n");
			return *this;
		}
		/** @function <<
		*   @brief 表示
		*/
		friend std::ostream& operator<<(std::ostream& os, const Vector& v)
		{
			return os << "(" << std::setw(2) << (int)v.x << ", " << std::setw(2) << (int)v.y << ")";
		}
	};
	/** @typedef Vectors
	*   @brief Vector構造体の動的配列
	*/
	typedef std::vector<Vector> Vectors;

	/** @union WallLog
	*   @brief 区画位置，方向，壁の有無を保持する構造体
	*/
	union WallLog {
		uint16_t all; /**< @brief 全フラグ参照用 */
		struct {
			int8_t x : 6;  /**< @brief 区画のx座標 */
			int8_t y : 6;  /**< @brief 区画のx座標 */
			uint8_t d : 3;  /**< @brief 方向 */
			uint8_t b : 1;  /**< @brief 壁の有無 */
		};
		WallLog(const Vector& v , const Dir& d, const bool b): x(v.x), y(v.y), d(d), b(b) {}
		WallLog(const int8_t x, const int8_t y, const Dir& d, const bool b): x(x), y(y), d(d), b(b) {}
		WallLog(const uint16_t all): all(all) {}
		WallLog() {}
		operator Vector(){ return Vector(x, y); }
	};
	/** @typedef WallLogs
	*   @brief WallLog構造体の動的配列
	*/
	typedef std::vector<WallLog> WallLogs;

	/** @class Maze
	*   @brief 迷路の壁情報を管理するクラス
	*   バックアップする際の情報量を最小限にするため，
	*   スタートとゴールの情報は持たず，壁の情報のみを保持する．
	*/
	class MazeWall {
	public:
		MazeWall() { reset(); } /**< @brief デフォルトのコンストラクタ */
		/** @brief 配列から迷路を読み込むコンストラクタ
		*   @param data 各区画16進表記の文字列配列
		*  例：{"abaf", "1234", "abab", "aaff"}
		*   @param east_origin true: 東から反時計回り，false: 北から時計回り に0bitから格納されている
		*/
		MazeWall(const char data[MAZE_SIZE+1][MAZE_SIZE+1], bool east_origin = true)
		{
			for(uint8_t y=0; y<MAZE_SIZE; y++)
			for(uint8_t x=0; x<MAZE_SIZE; x++){
				char c = data[MAZE_SIZE-y-1][x];
				uint8_t h = 0;
				if ('0' <= c && c <= '9') h = c-'0';
				else if('a'<=c && c<='f') h = c-'a'+10;
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
		/** @function reset
		*   @brief 迷路の初期化．壁を削除し，スタート区画を既知に
		*/
		void reset(const bool setStartWall = true)
		{
			for(int8_t i=0; i<MAZE_SIZE-1; i++){
				wall[0][i]=0;
				wall[1][i]=0;
				known[0][i]=0;
				known[1][i]=0;
			}
			if(setStartWall){
				updateWall(Vector(0,0), Dir::East, true); //< start cell
				updateWall(Vector(0,0), Dir::North, false); //< start cell
			}
		}
		/** @function isWall
		*   @brief 壁の有無を返す
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @return true: 壁あり，false: 壁なし
		*/
		bool isWall(const Vector& v, const Dir& d) const { return isWall(v.x, v.y, d); }
		bool isWall(const int8_t& x, const int8_t& y, const Dir& d) const
		{
			switch(d){
				case Dir::East:
				if(x<0 || x>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return true; //< 盤面外
				return wall[0][x] & (1<<y);
				case Dir::North:
				if(x<0 || x>MAZE_SIZE-1 || y<0 || y>MAZE_SIZE-2) return true; //< 盤面外
				return wall[1][y] & (1<<x);
				case Dir::West:
				if(x-1<0 || x-1>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return true; //< 盤面外
				return wall[0][x-1] & (1<<y);
				case Dir::South:
				if(x<0 || x>MAZE_SIZE-1 || y-1<0 || y-1>MAZE_SIZE-2) return true; //< 盤面外
				return wall[1][y-1] & (1<<x);
			}
			printf("Warning: invalid direction\n");
			return true;
		}
		/** @function setWall
		*   @brief 壁を更新をする
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @param b 壁の有無 true:壁あり，false:壁なし
		*/
		void setWall(const Vector& v, const Dir& d, const bool& b) { return setWall(v.x, v.y, d, b); }
		void setWall(const int8_t& x, const int8_t& y, const Dir& d, const bool& b)
		{
			switch(d){
				case Dir::East:
				if(x<0 || x>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return; //< 盤面外
				if(b) wall[0][x] |= (1<<y); else wall[0][x] &= ~(1<<y); return;
				case Dir::North:
				if(x<0 || x>MAZE_SIZE-1 || y<0 || y>MAZE_SIZE-2) return; //< 盤面外
				if(b) wall[1][y] |= (1<<x); else wall[1][y] &= ~(1<<x); return;
				case Dir::West:
				if(x-1<0 || x-1>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return; //< 盤面外
				if(b) wall[0][x-1] |= (1<<y); else wall[0][x-1] &= ~(1<<y); return;
				case Dir::South:
				if(x<0 || x>MAZE_SIZE-1 || y-1<0 || y-1>MAZE_SIZE-2) return; //< 盤面外
				if(b) wall[1][y-1] |= (1<<x); else wall[1][y-1] &= ~(1<<x); return;
			}
			printf("Warning: invalid direction\n");
		}
		/** @function isKnown
		*   @brief 壁が探索済みかを返す
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @return true: 探索済み，false: 未探索
		*/
		bool isKnown(const Vector& v, const Dir& d) const { return isKnown(v.x, v.y, d); }
		bool isKnown(const int8_t& x, const int8_t& y, const Dir& d) const
		{
			switch(d){
				case Dir::East:
				if(x<0 || x>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return true; //< 盤面外
				return known[0][x] & (1<<y);
				case Dir::North:
				if(x<0 || x>MAZE_SIZE-1 || y<0 || y>MAZE_SIZE-2) return true; //< 盤面外
				return known[1][y] & (1<<x);
				case Dir::West:
				if(x-1<0 || x-1>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return true; //< 盤面外
				return known[0][x-1] & (1<<y);
				case Dir::South:
				if(x<0 || x>MAZE_SIZE-1 || y-1<0 || y-1>MAZE_SIZE-2) return true; //< 盤面外
				return known[1][y-1] & (1<<x);
			}
			printf("Warning: invalid direction\n");
			return false;
		}
		/** @function setWall
		*   @brief 壁の既知を更新する
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @param b 壁の未知既知 true:既知，false:未知
		*/
		void setKnown(const Vector& v, const Dir& d, const bool& b) { return setKnown(v.x, v.y, d, b); }
		void setKnown(const int8_t& x, const int8_t& y, const Dir& d, const bool& b)
		{
			switch(d){
				case Dir::East:
				if(x<0 || x>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return; //< 盤面外
				if(b) known[0][x] |= (1<<y); else known[0][x] &= ~(1<<y); return;
				case Dir::North:
				if(x<0 || x>MAZE_SIZE-1 || y<0 || y>MAZE_SIZE-2) return; //< 盤面外
				if(b) known[1][y] |= (1<<x); else known[1][y] &= ~(1<<x); return;
				case Dir::West:
				if(x-1<0 || x-1>MAZE_SIZE-2 || y<0 || y>MAZE_SIZE-1) return; //< 盤面外
				if(b) known[0][x-1] |= (1<<y); else known[0][x-1] &= ~(1<<y); return;
				case Dir::South:
				if(x<0 || x>MAZE_SIZE-1 || y-1<0 || y-1>MAZE_SIZE-2) return; //< 盤面外
				if(b) known[1][y-1] |= (1<<x); else known[1][y-1] &= ~(1<<x); return;
			}
			printf("Warning: invalid direction\n");
		}
		/** @function canGo
		*   @brief 通過可能かどうかを返す
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @return true:既知かつ壁なし，false:それ以外
		*/
		bool canGo(const Vector& v, const Dir& d) const
		{
			return isKnown(v, d) && !isWall(v, d);
		}
		/** @function wallCount
		*   @brief 引数区画の壁の数を返す
		*   @param v 区画の座標
		*   @return 壁の数 0~4
		*/
		int8_t wallCount(const Vector& v) const
		{
			auto dirs = Dir::All();
			return std::count_if(dirs.begin(), dirs.end(), [&](const Dir& d){return isWall(v, d);});
		}
		/** @function unknownCount
		*   @brief 引数区画の未知壁の数を返す
		*   @param v 区画の座標
		*   @return 既知壁の数 0~4
		*/
		int8_t unknownCount(const Vector& v) const
		{
			auto dirs = Dir::All();
			return std::count_if(dirs.begin(), dirs.end(), [&](const Dir& d){return !isKnown(v, d);});
		}
		/** @function updateWall
		*   @brief 既知の壁と照らしあわせながら，壁を更新する関数
		*   @param v 区画の座標
		*   @param d 壁の方向
		*   @param b 壁の有無
		*   @return true: 正常に更新された, false: 既知の情報と不一致だった
		*/
		bool updateWall(const Vector& v, const Dir& d, const bool& b)
		{
			// 既知の壁と食い違いがあったら未知壁としてreturn
			if(isKnown(v, d) && isWall(v, d) != b){
				setWall(v, d, false);
				setKnown(v, d, false);
				return false;
			}
			// 未知壁なら壁情報を更新
			if(!isKnown(v, d)){
				setWall(v, d, b);
				setKnown(v, d, true);
			}
			return true;
		}
		/** @function print
		*   @brief 迷路の表示
		*   @param of output-stream
		*/
		void print(std::ostream& os = std::cout) const
		{
			printPath(os, Vector(0,0), {});
		}
		/** @function printPath
		*   @brief パス付の迷路の表示
		*   @param start パスのスタート座標
		*   @param dirs 移動方向の配列
		*   @param of output-stream
		*/
		void printPath(std::ostream& os, const Vector start, const Dirs& dirs) const
		{
			int steps[MAZE_SIZE][MAZE_SIZE]={0};
			Vector v = start;
			int counter = 1;
			for(const auto d: dirs){
				v = v.next(d);
				steps[v.y][v.x] = counter++;
			}
			for(int8_t y=MAZE_SIZE; y>=0; y--){
				if(y != MAZE_SIZE){
					os << '|';
					for(uint8_t x=0; x<MAZE_SIZE; x++){
						if(steps[y][x]!=0) os << C_YELLOW << std::setw(3) << steps[y][x] << C_RESET;
						else               os << "   ";
						os << (isKnown(x,y,Dir::East)?(isWall(x,y,Dir::East)?"|":" "):(C_RED "." C_RESET));
					}
					os << std::endl;
				}
				for(uint8_t x=0; x<MAZE_SIZE; x++) os << "+" << (isKnown(x,y,Dir::South)?(isWall(x,y,Dir::South)?"---":"   "):(C_RED " . " C_RESET));
				os << "+" << std::endl;
			}
		}
		void printPath(const Vector start, const Dirs& dirs) const
		{
			printPath(std::cout, start, dirs);
		}

	private:
		wall_size_t wall[2][MAZE_SIZE-1]; /**< 壁情報 */
		wall_size_t known[2][MAZE_SIZE-1]; /**< 既知壁情報 */
	};

	class Maze : public MazeWall {
	public:
		Maze(const Vectors& goals, const Vector start = Vector(0, 0))
		: MazeWall(), goals(goals), start(start) {}
		/** @constructor Maze
		*   @brief ファイル名から迷路をパースするコンストラクタ
		*   @param filename ファイル名
		*/
		Maze(const char* filename)
		{
			std::ifstream ifs(filename);
			parse(ifs);
		}
		/** @function reset
		*   @brief 迷路の初期化
		*/
		void reset()
		{
			MazeWall::reset();
			wallLogs.clear();
		}
		bool updateWall(const Vector& v, const Dir& d, const bool& b)
		{
			wallLogs.push_back(WallLog(v, d, b));
			if(!MazeWall::updateWall(v, d, b)) return false; //< 既知壁と食い違いがあった
			return true;
		}
		bool resetLastWall(const int num)
		{
			for(int i=0;i<num;i++){
				if(wallLogs.empty()) return true;
				auto wl = wallLogs.back();
				setWall(Vector(wl), wl.d, false);
				setKnown(Vector(wl), wl.d, false);
				wallLogs.pop_back();
			}
		}
		/** @function print
		*   @brief 迷路の表示
		*   @param of output-stream
		*/
		void print(std::ostream& os = std::cout) const
		{
			for(int8_t y=MAZE_SIZE; y>=0; y--){
				if(y != MAZE_SIZE){
					os << '|';
					for(uint8_t x=0; x<MAZE_SIZE; x++){
						auto v = Vector(x, y);
						if(v == start) os << " S ";
						else if(std::find(goals.begin(), goals.end(), v) != goals.end()) os << " G ";
						else               os << "   ";
						os << (isKnown(x,y,Dir::East)?(isWall(x,y,Dir::East)?"|":" "):".");
					}
					os << std::endl;
				}
				for(uint8_t x=0; x<MAZE_SIZE; x++) os << "+" << (isKnown(x,y,Dir::South)?(isWall(x,y,Dir::South)?"---":"   "):" . ");
				os << "+" << std::endl;
			}
		}
		/** @function parse
		*   @brief 迷路の文字列から壁をパースする
		*   @param is input-stream
		*/
		bool parse(std::istream& is)
		{
			reset();
			goals.clear();
			for(int8_t y=MAZE_SIZE; y>=0; y--){
				if(y!=MAZE_SIZE){
					is.ignore(10, '|'); //< 次の|が出てくるまでスキップ
					for(uint8_t x=0; x<MAZE_SIZE; x++) {
						is.ignore(1); //< " " 空欄分をスキップ
						char c = is.get();
						if     (c=='S') start = Vector(x, y);
						else if(c=='G') goals.push_back(Vector(x, y));
						is.ignore(1); //< " " 空欄分をスキップ
						c = is.get();
						if     (c=='|') MazeWall::updateWall(Vector(x, y), Dir::East, true);
						else if(c==' ') MazeWall::updateWall(Vector(x, y), Dir::East, false);
					}
				}
				for(uint8_t x=0; x<MAZE_SIZE; x++){
					is.ignore(10, '+'); //< 次の+が出てくるまでスキップ
					std::string s;
					s += (char)is.get();
					s += (char)is.get();
					s += (char)is.get();
					if     (s=="---") MazeWall::updateWall(Vector(x, y), Dir::South, true);
					else if(s=="   ") MazeWall::updateWall(Vector(x, y), Dir::South, false);
				}
			}
			return true;
		}

		void setGoals(const Vectors& goals) { this->goals=goals; }
		const Vectors& getGoals() const { return goals; }
		const Vector& getStart() const { return start; }
		const WallLogs& getWallLogs() const { return wallLogs; }

	private:
		Vectors goals;
		Vector start;
		WallLogs wallLogs;
	};
}
