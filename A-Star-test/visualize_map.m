function visualize_map(map, MAX_X, MAX_Y)
%This function visualizes the 2D grid map 
%consist of obstacles/start point/target point/optimal path.
set(gca, 'XTick', 0:1:MAX_X);
set(gca, 'YTick', 0:1:MAX_Y);
grid on;
% grid minor;
axis equal;        
axis ([0 MAX_X 0 MAX_Y ]);
hold on;
% obstacles
for obs_cnt = 2: size(map, 1) - 1
    fill([map(obs_cnt, 1),map(obs_cnt, 1),map(obs_cnt, 1)-1,map(obs_cnt, 1)-1,map(obs_cnt, 1)],...
         [map(obs_cnt, 2),map(obs_cnt, 2)-1,map(obs_cnt, 2)-1,map(obs_cnt, 2),map(obs_cnt, 2)],'k');
    hold on;
end
% start point
fill([map(1, 1),map(1, 1),map(1, 1)-1,map(1, 1)-1,map(1, 1)],...
     [map(1, 2),map(1, 2)-1,map(1, 2)-1,map(1, 2),map(1, 2)],'y');
hold on;
% target point
fill([map(size(map, 1)),map(size(map, 1)),map(size(map, 1))-1,map(size(map, 1))-1,map(size(map, 1))],...
     [map(size(map, 1), 2),map(size(map, 1), 2)-1,map(size(map, 1), 2)-1,map(size(map, 1), 2),map(size(map, 1), 2)],'g');
hold on;

end

