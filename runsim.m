%% Setup

close all
addpath('utils')
addpath('maps')

profile on

%% Simulation Parameters
%
% Define any additional parameters you may need here. Add the parameters to
% the robot and map structures to pass them to astar or rrt.
%
load 'robot.mat' robot

start = [0, 0, 0, 0, 0, 0];
goal =[1.4, 0, 0, 0, 0, 0];
map = loadmap('map6.txt');


%% Run the simulation

%% Run the simulation

% Turn the map into a C-space map
% cmap = getCMap(map,robot,[0.2,0.2,0.2],10);
% Solve the path problem
% tic
% [path, num] = astar(cmap, start, goal, false);
% toc
% OR
tic
[path, costsToGoal] = rrt_star_pp(map,start,goal);
toc
profile off

%% Plot the output
tic
plotLynxPath(map,path,10);
toc
profile viewer

f2 = figure();
plot(costsToGoal(1,:),costsToGoal(2,:) );
title('Cost to Goal over iterations')
xlabel('i th iteration')
ylabel('Length of path to goal')
