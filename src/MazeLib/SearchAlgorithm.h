/**
* @file SearchAlgorithm.h
* @brief マイクロマウスの迷路の探索アルゴリズムを扱うクラス
* @author KERI (Github: kerikun11)
* @date 2017.11.05
*/
#pragma once

#include "Maze.h"
#include "StepMap.h"

namespace MazeLib {
	/** @def FIND_ALL_WALL
	*   @brief 全探索するかどうか
	*   true: 全探索
	*   false: 最短になり得ないところは排除
	*/
	#define FIND_ALL_WALL 0
	/** @def SEARCHING_ADDITIALLY_AT_START
	*   @brief 追加探索状態で探索を始める(ゴールを急がない)
	*/
	#define SEARCHING_ADDITIALLY_AT_START 0

	/** @class SearchAlgorithm
	*   @brief 迷路探索アルゴリズムを司るクラス
	*/
	class SearchAlgorithm {
	public:
		/** @brief コンストラクタ
		*   @param maze 使用する迷路の参照
		*   @param goal ゴール区画の配列
		*/
		SearchAlgorithm(Maze& maze, Maze& idMaze, WallLogs& idWallLogs, const Vectors& goal)
		: maze(maze), idMaze(idMaze), idWallLogs(idWallLogs), goal(goal) {}
		/** @enum State
		*   @brief 探索状態を列挙
		*/
		enum State{
			START,									//< 初期位置，初期姿勢
			SEARCHING_FOR_GOAL,			//< ゴール区画を探索中
			SEARCHING_ADDITIONALLY,	//< 追加探索中
			BACKING_TO_START, 			//< スタートに戻っている
			REACHED_START,					//< スタートに戻ってきた
			IMPOSSIBLE,							//< ゴールにだどりつくことができないと判明した
			IDENTIFYING_POSITION,		//< 自己位置同定中
		};
		/** @function stateString
		*   @brief SearchAlgorithm::Stateの表示用文字列を返す関数
		*/
		static const char* stateString(const enum State s){
			static const char* str[]={
				"start",
				"Searching for Goal",
				"Searching Additionally",
				"Backing to Start",
				"Reached Start",
				"Impossible",
				"Identifying Position",
			};
			return str[s];
		}
		void replaceGoal(const Vectors& goal) {
			this->goal = goal;
		}
		/** @function isComplete
		*   @brief 最短経路が導出されているか調べる関数
		*/
		bool isComplete(){
			findShortestCandidates(candidates);
			return candidates.empty();
		}
		/** @function calcNextDirs
		*   @brief 次に行くべき方向を計算する
		*   @param pv 出発位置
		*   @param pd 出発方向
		*   @param state 出発時の探索状態
		*   @return true: 成功, false: 失敗
		*/
		bool calcNextDirs(enum State& state, Vector& pv, const Dir& pd, Dirs& nextDirs, Dirs& nextDirCandidates, const bool isForceBackToStart) {
			nextDirs.clear();
			nextDirCandidates.clear();

			if(state == START){
				state = SEARCHING_FOR_GOAL;
				#if SEARCHING_ADDITIALLY_AT_START
				state = SEARCHING_ADDITIONALLY;
				#endif
				// 強制帰還がリクエストされていたらとりあえずゴールを目指す
				if(isForceBackToStart) state = SEARCHING_FOR_GOAL;
			}

			if(state == IDENTIFYING_POSITION){
				Vector ans;
				int cnt = countIdentityCandidates(idWallLogs, ans);
				matchCount = cnt;
				if(cnt == 1) {
					pv = pv - idStartVector() + ans;
					state = SEARCHING_FOR_GOAL;
				} else if(cnt == 0){
					return false;
				} else {
					candidates.clear();
					for(auto v: {Vector(MAZE_SIZE-1, MAZE_SIZE-1), Vector(MAZE_SIZE-1, 0), Vector(0, MAZE_SIZE-1)}){
						if(idMaze.unknownCount(v)){
							candidates.push_back(v);
							break;
						}
					}
					stepMap.updateSimple(idMaze, candidates, false);
					return stepMap.calcNextDirs(idMaze, pv, pd, nextDirs, nextDirCandidates);
				}
			}

			if(state == SEARCHING_FOR_GOAL){
				// state = SEARCHING_ADDITIONALLY;
				// ゴール区画が探索済みなら次のstateへ
				const auto unknownGoal = std::find_if(goal.begin(), goal.end(), [&](const Vector& v){ return maze.unknownCount(v); });
				if(unknownGoal == goal.end()){
					state = SEARCHING_ADDITIONALLY;
				} else {
					// ゴールを目指して探索
					stepMap.updateSimple(maze, {*unknownGoal}, false);	//< ゴール，壁なし，斜めなし
					return stepMap.calcNextDirs(maze, pv, pd, nextDirs, nextDirCandidates);
				}
			}

			if(state == SEARCHING_ADDITIONALLY){
				// 強制帰還がリクエストされていたら帰る
				if(isForceBackToStart) state = BACKING_TO_START;
				// 最短になりうる区画の洗い出し
				findShortestCandidates(candidates);
				if(candidates.empty()){
					state = BACKING_TO_START;
				}else{
					stepMap.updateSimple(maze, candidates, false);
					return stepMap.calcNextDirs(maze, pv, pd, nextDirs, nextDirCandidates);
				}
			}

			if(state == BACKING_TO_START){
				if(pv == start) {
					state = REACHED_START;
				}else{
					stepMap.updateSimple(maze, {start}, false);
					return stepMap.calcNextDirs(maze, pv, pd, nextDirs, nextDirCandidates);
				}
			}

			if(state == REACHED_START){
				return true;
			}

			// ここには達しない
			return false;
		}
		bool calcNextDirsInAdvance(enum State& state, Vector& pv, const Dir& pd, Dirs& nextDirs, Dirs& nextDirCandidates, const bool isForceBackToStart = false) {
			calcNextDirs(state, pv, pd, nextDirs, nextDirCandidates, isForceBackToStart);
			auto v = pv; for(auto d: nextDirs) v = v.next(d);
			Dirs ndcs;
			WallLogs cache;
			while(1){
				if(nextDirCandidates.empty()) break;
				const Dir d = nextDirCandidates[0];
				ndcs.push_back(d);
				if(maze.isKnown(v, d)) break; //< 既知なら終わり
				cache.push_back(WallLog(v, d, false));
				maze.setWall (v, d, true);
				maze.setKnown (v, d, true);
				State tmp_state = state;
				Dirs tmp_nds;
				calcNextDirs(tmp_state, v, d, tmp_nds, nextDirCandidates, isForceBackToStart);
				if(!tmp_nds.empty()) {
					nextDirCandidates = tmp_nds;
				}
			}
			for(auto wl: cache) {
				maze.setWall (Vector(wl), wl.d, false);
				maze.setKnown(Vector(wl), wl.d, false);
			}
			nextDirCandidates = ndcs;
			return !ndcs.empty();
		}
		/** @function calcShortestDirs
		*   @brief 最短経路を導出
		*   @return 成功 or 失敗
		*/
		bool calcShortestDirs(Dirs& shortestDirs, const bool diagonal = true){
			stepMap.update(maze, goal, true, diagonal);
			// stepMap.update(maze, goal, false, diagonal); //< for debug
			shortestDirs.clear();
			auto v = start;
			Dir dir = Dir::North;
			auto prev_dir = dir;
			while(1){
				step_t min_step = MAZE_STEP_MAX;
				const auto& dirs = dir.ordered(prev_dir);
				prev_dir = dir;
				for(const auto& d: dirs){
					if(!maze.canGo(v, d)) continue;
					step_t next_step = stepMap.getStep(v.next(d));
					if(min_step > next_step) {
						min_step = next_step;
						dir = d;
					}
				}
				if(stepMap.getStep(v) <= min_step) return false; //< 失敗
				shortestDirs.push_back(dir);
				v = v.next(dir);
				if(stepMap.getStep(v) == 0) break; //< ゴール区画
			}
			// ゴール区画を行けるところまで直進する
			bool loop = true;
			while(loop){
				loop = false;
				Dirs dirs;
				switch (Dir(dir-prev_dir)) {
					case Dir::Left: dirs = {dir.getRelative(Dir::Right), dir}; break;
					case Dir::Right: dirs = {dir.getRelative(Dir::Left), dir}; break;
					case Dir::Forward: default: dirs = {dir}; break;
				}
				if(!diagonal) dirs = {dir};
				for(const auto& d: dirs){
					if(maze.canGo(v, d)){
						shortestDirs.push_back(d);
						v = v.next(d);
						prev_dir = dir;
						dir = d;
						loop = true;
						break;
					}
				}
			}
			return true;
		}
		void printMap(const State state, const Vector& v, const Dir& d) const {
			for(int i=0; i<MAZE_SIZE*2; i++) printf("\x1b[A");
			switch (state) {
				case IDENTIFYING_POSITION:
				return stepMap.print(idMaze, v, d);
				// return stepMap.print(maze, v+Vector(3,-4), d);
				default:
				return stepMap.print(maze, v, d);

			}
		}
		// const StepMap& getStepMap() const { return stepMap; }
		static const Vector& idStartVector() {
			static auto v = Vector(MAZE_SIZE/2, MAZE_SIZE/2);
			return v;
		}
		int matchCount = 0;

	private:
		Maze& maze; /**< 使用する迷路の参照 */
		Maze& idMaze;
		WallLogs& idWallLogs;
		StepMap stepMap; /**< 使用するステップマップ */
		const Vector start{0, 0}; /**< スタート区画を定義 */
		Vectors goal; /**< ゴール区画を定義 */
		Vectors candidates; /**< 最短経路上になり得る候補を入れるコンテナ */

		/** @function findShortestCandidates
		*   @brief ステップマップにより最短経路上になりうる区画を洗い出す
		*/
		bool findShortestCandidates(Vectors& candidates) {
			candidates.clear();
			// 斜めありなしの双方の最短経路上を候補とする
			for(const bool diagonal: {true, false}){
				stepMap.update(maze, goal, false, diagonal);
				auto v = start;
				Dir dir = Dir::North;
				auto prev_dir = dir;
				while(1){
					step_t min_step = MAZE_STEP_MAX;
					const auto& dirs = dir.ordered(prev_dir);
					prev_dir = dir;
					for(const auto& d: dirs){
						if(maze.isWall(v, d)) continue;
						step_t next_step = stepMap.getStep(v.next(d));
						if(min_step > next_step) {
							min_step = next_step;
							dir = d;
						}
					}
					if(stepMap.getStep(v) <= min_step) return false; //< 失敗
					if(maze.unknownCount(v)) candidates.push_back(v);
					v = v.next(dir);
					if(stepMap.getStep(v) == 0) break; //< ゴール区画
				}
				// ゴール区画を行けるところまで直進する
				bool loop = true;
				while(loop){
					loop = false;
					Dirs dirs;
					switch (Dir(dir-prev_dir)) {
						case Dir::Left: dirs = {dir.getRelative(Dir::Right), dir}; break;
						case Dir::Right: dirs = {dir.getRelative(Dir::Left), dir}; break;
						case Dir::Forward: default: dirs = {dir}; break;
					}
					if(!diagonal) dirs = {dir};
					for(const auto& d: dirs){
						if(!maze.isWall(v, d)){
							if(maze.unknownCount(v)) candidates.push_back(v);
							v = v.next(d);
							prev_dir = dir;
							dir = d;
							loop = true;
							break;
						}
					}
				}
			}
			return true; //< 成功
		}
		int countIdentityCandidates(const WallLogs idWallLogs, Vector& ans) const {
			int cnt = 0;
			for(int x=-MAZE_SIZE/2+1; x<MAZE_SIZE/2; x++)
			for(int y=-MAZE_SIZE/2+1; y<MAZE_SIZE/2; y++) {
				const Vector offset(x, y);
				int diffs=0;
				int matchs=0;
				int unknown=0;
				for(auto wl: idWallLogs){
					Vector v(wl.x, wl.y);
					Dir d = wl.d;
					if(maze.isKnown(v+offset, d) && maze.isWall(v+offset, d) != wl.b) diffs++;
					if(maze.isKnown(v+offset, d) && maze.isWall(v+offset, d) == wl.b) matchs++;
					if(!maze.isKnown(v+offset, d)) unknown++;
				}
				int size = idWallLogs.size();
				if(diffs <= 4) {
					if(size<4 || unknown<size/2 || matchs>size/2) {
						ans = idStartVector() + offset;
						cnt++;
					}
				}
			}
			return cnt;
		}
	};
}
