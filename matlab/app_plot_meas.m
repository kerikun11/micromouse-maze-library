%% Micromouse; Log File Loader and Visualizer
% Author: Ryotaro Onuki
% Created_at:  2019.05.10
% Modified_at: 2019.05.10

%% cleaning
clear;

%% Load
data = readtable('../build/measurement.csv');

%%
c = categorical(data.Var1);
bar(c, data.Var2);
