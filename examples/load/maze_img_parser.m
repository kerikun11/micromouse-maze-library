%% 入力
[filename, pathname] = uigetfile({'*.jpg;*.png;*.gif'}, 'Select the Maze Imgae');

%% 二値化＆色反転(壁を1にするため)
original = imread([pathname, filename]);
bw = imbinarize(rgb2gray(original));
bw = imcomplement(bw);
imshow(bw);

%% 迷路の輪郭を抽出
colsum = sum(bw);
rowsum = sum(bw, 2);
[wM, we] = max(colsum(1:end / 2));
[eM, ee] = max(colsum(end / 2:end));
[nM, ne] = max(rowsum(1:end / 2));
[sM, se] = max(rowsum(end / 2:end));
trim = bw(ne - 1:(se + size(rowsum, 1) / 2) + 1, we - 1:(ee + size(colsum, 2) / 2) + 1);
trim = imresize(trim, [1600 1600]);

%% ノイズの除去と壁の膨張
trim = imopen(trim, ones(9)); % 白点線を除去
trim = imdilate(trim, ones(12)); % 白線を太くする
imshow(trim);

%% 迷路サイズを検出
trimsum = sum(trim);
trimsum = trimsum < sum(trimsum) / length(trimsum);
maze_size = sum(([trimsum 0] - [0 trimsum]) > 0);
msgbox(sprintf('迷路サイズは %d です', maze_size)); % 迷路サイズの表示

%% 壁の抽出
segsize = size(trim) / maze_size;
vwall = trim(round(segsize(1) / 2:segsize(1):end), round(1:segsize(2):end - segsize(2) / 3));
hwall = trim(round(1:segsize(1):end - segsize(1) / 3), round(segsize(2) / 2:segsize(2):end));
vwall = [vwall, ones(maze_size, 1)];
hwall = [hwall; ones(1, maze_size)];

%% 壁の合成
wall = 1 * vwall(:, 2:end); % 東 0 bit
wall = wall + 2 * hwall(1:end - 1, :); % 北 1 bit
wall = wall + 4 * vwall(:, 1:end - 1); % 西 2 bit
wall = wall + 8 * hwall(2:end, :); % 南 3 bit

%% ファイルに保存
output = reshape(sprintf('%x', wall), maze_size, maze_size);
output = ['"'* ones(maze_size, 1) output '",'.* ones(maze_size, 1)];
output_dir = 'output';
new_filename = sprintf('%s/%s.txt', output_dir, filename);
dlmwrite(new_filename, output, 'precision', '%c', 'delimiter', '');
