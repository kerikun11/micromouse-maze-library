## アルゴリズムの概要

以下で説明するアルゴリズムを用いることで，どんな迷路に対しても少なくともひとつの経路を見つけ出すことができる．
ただし，目的地が封鎖されていて経路が存在しない場合はその事実を検出して終了する．

--------------------------------------------------------------------------------

### 迷路探索に必要な処理

マイクロマウスで迷路探索を行うためには，主に以下の処理が必要となる．

- 迷路上にある機体の区画位置と進行方向を識別する処理 (MazeLib::Position, MazeLib::Direction)
- 迷路上の全壁の有無と既知未知を管理する処理 (MazeLib::WallIndex, MazeLib::Maze)
- 迷路上のある区画からある区画(の集合)への移動経路を導出する処理 (MazeLib::StepMap)
- スタート区画からゴール区画(の集合)への最短経路を導出する処理 (※)

※初期段階では最短経路導出処理は移動経路導出処理で代用できる．

--------------------------------------------------------------------------------

### 探索走行アルゴリズム

未知の迷路を探索し，スタートからゴールまでの最短経路を発見するアルゴリズム．

1. ゴール区画までの往路探索走行
   1. 自己位置からゴール区画までの移動経路を，未知壁は壁なしとして導出する．
      1. 経路が存在しない場合は異常終了とする．
   2. 上記の経路を未知壁を含む区画に当たるまで進む．
   3. センサによって壁を確認し，迷路情報を更新する．
   4. 現在位置がゴールでなければ1へ戻る．
2. 最短経路を見つける追加探索走行
   1. スタート区画からゴール区画までの最短経路を，未知壁は壁なしとして導出する．
      1. 経路が存在しない場合は異常終了とする．
   2. 最短経路上の未知壁を含む区画を目的地とする．目的地が空なら終了する．
   3. 自己位置から目的地までの移動経路を，未知壁は壁なしとして導出する．
   4. 上記の経路を未知壁を含む区画に当たるまで進む．
   5. センサによって壁を確認し，迷路情報を更新する．
   6. 1へ戻る．
3. スタートに戻る走行
   1. 自己位置からスタート区画までの経路を，未知壁は壁ありとして導出する．
   2. 上記の経路を進んでスタート区画に到達する．

### 移動経路導出アルゴリズム

探索中に用いる，「ある始点区画」から「ある目的区画集合」への「移動経路」導出処理．

1. 準備
   1. 迷路上の全区画のステップマップを用意する．
2. 展開
   1. 目的区画に含まれる区画のステップを0とする．
   2. そこから壁がない方向の区画へ進むたびにステップを1ずつ増やし，ステップマップを更新する．
   3. ステップの変更がなくなるまでステップマップを再帰的に更新する．
3. 経路導出
   1. 始点区画からステップが小さくなる方向へ順次進んでいくとやがて目的区画のひとつにたどりつき，それが移動経路となる．
   2. 目的区画にたどり着くことなく，ステップが小さくなる方向が存在しなくなった場合，その迷路に解はないので終了する．

### 最短経路導出アルゴリズム

探索中や最短走行前に用いる，「スタート区画」から「ゴール区画の集合」への「最短経路」の導出処理．

一番簡単な実装としては，
移動経路導出アルゴリズムの始点をスタート区画に，
目的区画をゴールにすればよい．
しかしながら，それだけだとターンの多い経路になりがちなので，
直線部分の加速や各ターンのコストを考慮した最短経路が導出できるとよい．

ここでは台形加速を考慮した直線コストのステップマップを紹介する．(斜め走行および各ターンのコストは考慮していない)

1. 準備
   1. 迷路上の全区画のステップマップを用意する．
   2. 台形加速で n マスの移動するときにかかる時間を列挙したコストテーブルを用意する．
   3. コストテーブルの全要素から1マス直線のコストを引き，さらに1マスのスラロームターンにかかる時間を足す．
2. 展開
   1. 目的区画に含まれる区画のステップを0とする．
   2. そこから壁がない方向の区画へ，直線で行けるところまでコストテーブルの値を用いてステップを更新する．
   3. ステップの変更がなくなるまでステップマップを再帰的に更新する．
3. 経路導出
   1. 始点区画からステップが小さくなる方向へ順次進んでいくとやがて目的区画のひとつにたどりつき，それが移動経路となる．
   2. 目的区画にたどり着くことなく，ステップが小さくなる方向が存在しなくなった場合，その迷路に解はないので終了する．