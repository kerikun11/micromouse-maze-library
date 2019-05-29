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
value = rawdata(1, :);
walllog = rawdata(2, :);

%% Visualization
%% Velocity
figure(figindex); figindex = figindex + 1;
subplotNum = 2; subplotIndex = 1;

subplot(subplotNum, 1, subplotIndex); hold on;
subplotIndex = subplotIndex + 1;
plot(value); grid on;
title('Calculation Time');
xlabel('Search Step');
ylabel('Calculation Time [us]');

subplot(subplotNum, 1, subplotIndex); hold on;
subplotIndex = subplotIndex + 1;
plot(walllog); grid on;
title('The Number of the Known Wall');
xlabel('Search Step');
ylabel('The Number of the Known Wall');

%% save
fig = gcf;
fig.Position(3) = 960;
fig.Position(4) = 720;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
print(fig, [datestr(datetime('now'), 'yymmdd-HHMMSS') '.pdf'], '-dpdf');
