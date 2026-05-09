% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;
addpath('A_star')

% Environment map in 2D space 
xStart = 2.0;
yStart =  2.0;
xTarget = 19.0;
yTarget = 2;
MAX_X = 20;
MAX_Y = 20;
%map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);

% Waypoint Generator Using the A* 
%[path,OPEN,CLOSED,k] = A_star_search(map, MAX_X,MAX_Y);
% visualize the 2D grid map
%visualize_map(map, path, OPEN);

%%
% BINN全覆盖路径规划

%  1. 调用新生成的固定障碍物地图，不再使用随机地图
map = fixed_obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);

%  2. 运行算法，注意现在多接收了一个 x_act_history 参数用于独立渲染 3D
[path, visit_nodes, deadlocks, x_act_history] = binn_ccpp(map, MAX_X, MAX_Y);

%  3. 调用全新的专用可视化函数
Binn_visualize_results(map, path, visit_nodes, deadlocks, x_act_history, MAX_X, MAX_Y);

%%
% save map
% save('Data/map.mat', 'map', 'MAX_X', 'MAX_Y');
% fprintf('加入的节点数量: %d\n', size(OPEN,1));
% fprintf('已扩展的节点数量: %d\n', size(CLOSED,1)-k+1);