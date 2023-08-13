% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;

set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [500, 50, 700, 700]);

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 30.0;
yTarget = 30.0;
MAX_X = 30;
MAX_Y = 30;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);

% visualize the 2D grid map
visualize_map(map, MAX_X, MAX_Y);

% Waypoint Generator Using the A* 
path = A_star_search(map, MAX_X,MAX_Y);

%optimal path
for path_cnt = 2:size(path,1)-1
    scatter(path(path_cnt,1),path(path_cnt,2),'b');
    hold on;
end
if (~isempty(path))
    plot(path(:,1),path(:,2),'m-','LineWidth',4);
end
