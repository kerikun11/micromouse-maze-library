%% 入力
maze_size = 32;
file_name = 'MM2016rec_HX.jpg';

%% 二値化＆色反転(壁を1にするため)
original = imread(file_name);
bw = imbinarize(rgb2gray(original));
bw = imcomplement(bw);

%% 迷路の輪郭を抽出
colsum = sum(bw);
rowsum = sum(bw,2);
[wM, we] = max(colsum(1:end/2));
[eM, ee] = max(colsum(end/2:end));
[nM, ne] = max(rowsum(1:end/2));
[sM, se] = max(rowsum(end/2:end));
trim = bw(ne:(se+size(rowsum,1)/2), we:(ee+size(colsum,2)/2));

%% ノイズの除去と壁の膨張
trim = imopen(trim, ones(2,2));
trim = imdilate(trim, ones(6,6));
imshow(trim);

%% 壁の抽出
segsize = size(trim)/maze_size;
vwall = trim(round(segsize(1)/2:segsize(1):end), round(1:segsize(2):end-segsize(2)/3));
hwall = trim(round(1:segsize(1):end-segsize(1)/3), round(segsize(2)/2:segsize(2):end));
vwall = [vwall, ones(maze_size, 1)];
hwall = [hwall; ones(1, maze_size)];

%% 壁の合成
wall =        1*vwall(:, 2:end);
wall = wall + 2*hwall(1:end-1, :);
wall = wall + 4*vwall(:, 1:end-1);
wall = wall + 8*hwall(2:end, :);

%% ファイルに保存
new_file_name = sprintf('%s.txt', file_name);
dlmwrite(new_file_name, wall, 'precision', '%x', 'delimiter', '');
