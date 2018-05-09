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
		SearchAlgorithm(Maze& maze, const Vectors& goal) : maze(maze),
		goal(goal) {}
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
			FAILED_TO_IDENTIFY,			//< 自己位置同定失敗
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
		/** @function calcNextDirs
		*   @brief 次に行くべき方向を計算する
		*   @param pv 出発位置
		*   @param pd 出発方向
		*   @param state 出発時の探索状態
		*/
		bool calcNextDirs(enum State& state, const Vector& pv, const Dir& pd, Dirs& nextDirs, Dirs& nextDirsInAdvance, const bool isForceBackToStart = false) {
			if(state == START){
				state = SEARCHING_FOR_GOAL;
				#if SEARCHING_ADDITIALLY_AT_START
				state = SEARCHING_ADDITIONALLY;
				#endif
				// 強制帰還がリクエストされていたらゴールを目指す
				if(isForceBackToStart) state = SEARCHING_FOR_GOAL;
			}

			if(state == SEARCHING_FOR_GOAL){
				// state = SEARCHING_ADDITIONALLY;
				// ゴール区画が探索済みなら次のstateへ
				const auto unknownGoal = std::find_if(goal.begin(), goal.end(), [&](const Vector& v){ return maze.unknownCount(v); });
				if(unknownGoal == goal.end()){
					state = SEARCHING_ADDITIONALLY;
				} else {
					// ゴールを目指して探索
					stepMapGoal.update(maze, goal, false, false);
					return stepMapGoal.calcNextDirs(maze, pv, pd, nextDirs, nextDirsInAdvance);
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
					stepMapCandidates.update(maze, candidates, false, false);
					return stepMapCandidates.calcNextDirs(maze, pv, pd, nextDirs, nextDirsInAdvance);
				}
			}

			if(state == BACKING_TO_START){
				if(pv == start) {
					state = REACHED_START;
				}else{
					stepMapStart.update(maze, {start}, false, false);
					return stepMapStart.calcNextDirs(maze, pv, pd, nextDirs, nextDirsInAdvance);
				}
			}

			if(state == REACHED_START){
				nextDirs.clear();
				nextDirsInAdvance.clear();
				return true;
			}

			if(state == IDENTIFYING_POSITION){
				Vector ans;
				int cnt = findIndentifyCandidate(idWallLogs, ans, candidates);
				Maze idMaze;
				for(auto& wl: idWallLogs) idMaze.updateWall(Vector(wl), wl.d, wl.b);
				stepMapCandidates.update(idMaze, candidates, false, false);
				stepMapCandidates.calcNextDirs(maze, pv, pd, nextDirs, nextDirsInAdvance);
			}

			return state == REACHED_START;
		}
		/** @function calcShortestDirs
		*   @brief 最短経路を導出
		*   @return 成功 or 失敗
		*/
		bool calcShortestDirs(Dirs& shortestDirs, const bool diagonal = true){
			stepMapGoal.update(maze, goal, true, diagonal);
			// stepMapGoal.update(maze, goal, false, diagonal); //< for debug
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
					step_t next_step = stepMapGoal.getStep(v.next(d));
					if(min_step > next_step) {
						min_step = next_step;
						dir = d;
					}
				}
				if(stepMapGoal.getStep(v) <= min_step) return false; //< 失敗
				shortestDirs.push_back(dir);
				v = v.next(dir);
				if(stepMapGoal.getStep(v) == 0) break; //< ゴール区画
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
		void printMap(const State& state, const Vector& v, const Dir& d) const {
			for(int i=0; i<MAZE_SIZE*2; i++) printf("\x1b[A");
			switch(state){
				case SearchAlgorithm::START:
				case SearchAlgorithm::SEARCHING_FOR_GOAL:
				stepMapGoal.print(maze, v, d);
				break;
				case SearchAlgorithm::SEARCHING_ADDITIONALLY:
				stepMapCandidates.print(maze, v, d);
				break;
				case SearchAlgorithm::BACKING_TO_START:
				stepMapStart.print(maze, v, d);
				break;
				case SearchAlgorithm::REACHED_START:
				case SearchAlgorithm::IMPOSSIBLE:
				default:
				stepMapGoal.print(maze, v, d);
				break;
			}
		}

	private:
		Maze& maze; /**< 使用する迷路の参照 */
		Maze idMaze;
    WallLogs idWallLogs;
		StepMap stepMapGoal; /**< 使用するステップマップ */
		StepMap stepMapStart; /**< 使用するステップマップ */
		StepMap stepMapCandidates; /**< 使用するステップマップ */
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
				stepMapGoal.update(maze, goal, false, diagonal);
				auto v = start;
				Dir dir = Dir::North;
				auto prev_dir = dir;
				while(1){
					step_t min_step = MAZE_STEP_MAX;
					const auto& dirs = dir.ordered(prev_dir);
					prev_dir = dir;
					for(const auto& d: dirs){
						if(maze.isWall(v, d)) continue;
						step_t next_step = stepMapGoal.getStep(v.next(d));
						if(min_step > next_step) {
							min_step = next_step;
							dir = d;
						}
					}
					if(stepMapGoal.getStep(v) <= min_step) return false; //< 失敗
					if(maze.unknownCount(v)) candidates.push_back(v);
					v = v.next(dir);
					if(stepMapGoal.getStep(v) == 0) break; //< ゴール区画
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
		int findIndentifyCandidate(const WallLogs idWallLogs, Vector& ans, Vectors& findCandidates) {
			findCandidates.clear();
			int cnt = 0;
			for(int x=-MAZE_SIZE+1; x<MAZE_SIZE; x++)
			for(int y=-MAZE_SIZE+1; y<MAZE_SIZE; y++) {
				const Vector offset(x, y);
				int diffs=0;
				for(auto wl: idWallLogs){
					Vector v(wl.x, wl.y);
					Dir d = wl.d;
					if(maze.isKnown(v+offset, d) && maze.isWall(v+offset, d) != wl.b) diffs++;
				}
				if(diffs == 0) {
					cnt++;
					ans = offset;
					if(ans.x>=0 && ans.x<MAZE_SIZE && ans.y>=0 && ans.y<MAZE_SIZE) candidates.push_back(ans);
				}
			}
			return cnt;
		}
	};
}
