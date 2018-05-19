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

	/** @class SearchAlgorithm
	*   @brief 迷路探索アルゴリズムを司るクラス
	*/
	class SearchAlgorithm {
	public:
		/** @brief コンストラクタ
		*   @param maze 使用する迷路の参照
		*   @param goal ゴール区画の配列
		*/
		SearchAlgorithm(Maze& maze, const Vectors& goal, const Vector& start, StepMap& stepMap)
		: maze(maze), goal(goal), start(start), stepMap(stepMap) {}
		/** @enum Status
		*   @brief 進むべき方向の計算結果
		*/
		enum Status{
			Processing,
			Reached,
			Error,
		};
		/** @function isComplete
		*   @brief 最短経路が導出されているか調べる関数
		*/
		bool isComplete(){
			Vectors candidates;
			findShortestCandidates(candidates);
			return candidates.empty();
		}
		enum Status calcNextDirsSearchForGoal(const Vector& cv, const Dir& cd, Dirs& nextDirsKnown, Dirs& nextDirCandidates){
			Vectors candidates;
			for(auto v: goal) if(maze.unknownCount(v)) candidates.push_back(v); //< ゴール区画の未知区画を洗い出す
			if(candidates.empty()) return Reached;
			stepMap.calcNextDirs(maze, candidates, cv, cd, nextDirsKnown, nextDirCandidates);
			return nextDirCandidates.empty() ? Error : Processing;
		}
		enum Status calcNextDirsSearchAdditionally(const Vector& cv, const Dir& cd, Dirs& nextDirsKnown, Dirs& nextDirCandidates){
			Vectors candidates;
			findShortestCandidates(candidates); //< 最短になりうる区画の洗い出し
			if(candidates.empty()) return Reached;
			stepMap.calcNextDirs(maze, candidates, cv, cd, nextDirsKnown, nextDirCandidates);
			return nextDirCandidates.empty() ? Error : Processing;
		}
		enum Status calcNextDirsBackingToStart(const Vector& cv, const Dir& cd, Dirs& nextDirsKnown, Dirs& nextDirCandidates){
			const auto v = stepMap.calcNextDirs(maze, {start}, cv, cd, nextDirsKnown, nextDirCandidates);
			if(v == start) return Reached;
			return nextDirCandidates.empty() ? Error : Processing;
		}
		enum Status calcNextDirsGoingToGoal(const Vector& cv, const Dir& cd, Dirs& nextDirsKnown, Dirs& nextDirCandidates){
			const auto v = stepMap.calcNextDirs(maze, goal, cv, cd, nextDirsKnown, nextDirCandidates);
			auto it = std::find_if(goal.begin(), goal.end(), [v](const Vector nv){ return v==nv; });
			if(it != goal.end()) return Reached;
			return nextDirCandidates.empty() ? Error : Processing;
		}
		enum Status calcNextDirsPositionIdentification(Maze& idMaze, WallLogs& idWallLogs, Vector& cv, const Dir& cd, Dirs& nextDirsKnown, Dirs& nextDirCandidates, int& matchCount){
			Vector ans;
			int cnt = countIdentityCandidates(idWallLogs, ans);
			matchCount = cnt;
			if(cnt == 1) {
				cv = cv - idStartVector + ans;
				return Reached;
			} else if(cnt == 0){
				return Error;
			}
			Vectors candidates;
			for(int8_t x=0; x<MAZE_SIZE; ++x) for(int8_t y=0; y<MAZE_SIZE; ++y) if(idMaze.unknownCount(Vector(x,y))) candidates.push_back(Vector(x,y));
			stepMap.calcNextDirs(idMaze, candidates, cv, cd, nextDirsKnown, nextDirCandidates);
			return nextDirCandidates.empty() ? Error : Processing;
		}
		const Vector& getIdStartVector() const { return idStartVector; }
		void setIdStartVector(const Vector& v) { idStartVector=v; }

	private:
		Maze& maze; /**< 使用する迷路の参照 */
		const Vectors& goal; /**< ゴール区画を定義 */
		const Vector& start; /**< スタート区画を定義 */
		StepMap& stepMap; /**< 使用するステップマップ */
		Vector idStartVector;

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
						case Dir::Front: default: dirs = {dir}; break;
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
			for(int x=-MAZE_SIZE/2; x<MAZE_SIZE/2; x++)
			for(int y=-MAZE_SIZE/2; y<MAZE_SIZE/2; y++) {
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
					// if(size<4 || unknown<size/2 || matchs>size/2) {
					ans = idStartVector + offset;
					cnt++;
					// }
				}
			}
			return cnt;
		}
	};
}
