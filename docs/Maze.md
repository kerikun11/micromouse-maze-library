# Maze Class

MazeLib::Maze

```flow
start=>start: スタート
end=>end: 終了
search_for_goal=>operation: ゴール区画探索
search_for_shortest=>operation: 最短経路探索
go_back_to_start=>operation: スタート帰還

start->search_for_goal->search_for_shortest->go_back_to_start->end
```
