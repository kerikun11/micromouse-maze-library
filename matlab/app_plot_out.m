%% Micromouse; Log File Loader and Visualizer
% Author: Ryotaro Onuki
% Created_at:  2019.05.10
% Modified_at: 2019.05.10

%% cleaning
clear;
% close all;
set(groot, 'DefaultTextInterpreter', 'Latex');
set(groot, 'DefaultLegendInterpreter', 'Latex');
set(groot, 'DefaultAxesFontSize', 14);
set(groot, 'DefaultLineLineWidth', 1.5);
figindex = 1;

%% Select a Log File
[filename, pathname] = uigetfile({'*'}, 'Select a Log File');
fprintf('Log File: %s\n', filename);

%% Load Data
rawdata = dlmread([pathname filename]);

%% Triming and Preprocess
rawdata = rawdata';

%% extract
value = rawdata(:, :);

%% Visualization
%% Velocity
figure(figindex); figindex = figindex + 1;
subplotNum = 1; subplotIndex = 1;

subplot(subplotNum, 1, subplotIndex); hold on;
subplotIndex = subplotIndex + 1;
plot(value); grid on;
title('Calculation Time');
xlabel('Search Step');
ylabel('Calculation Time [us]');

%% save
fig = gcf;
% fig.Position(3) = 480; fig.Position(4) = 360;
fig.PaperPositionMode = 'auto';
fig.PaperSize = [fig.PaperPosition(3) fig.PaperPosition(4)];
date_time_str = datestr(datetime('now'), 'yymmdd-HHMMSS');
outdir = 'output/';
[~, ~] = mkdir(outdir);
warning('off', 'all');
print(fig, [outdir date_time_str '.pdf'], '-dpdf');
print(fig, [outdir date_time_str '.png'], '-dpng');
savefig([outdir date_time_str '.fig']);
warning('on', 'all');
