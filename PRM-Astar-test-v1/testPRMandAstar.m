%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
%%----------------------------------------------
clc
clear all
close all
%% 1. Run PRM
%get the graph by running PRM and map lot
%V is all the qs'
qset=[3 19;3 19];  % 1st column: q_initial, 2nd: q_goal
n=50; % 随机生成的节点数
k=20; % 每个节点最大允许的邻近子节点数量
% 多边形表示的障碍物
Q1=[1 2 2 1;1 1 2 2];   % 障碍物
Q2=[10 15 15 10;
    10 10 15 15];   % 障碍物
Q3=[5 10 10 5;
    5 5 10 10];   % 障碍物
Q4=[2 4 4 2;14 14 16 16];   % 障碍物
Q5=[4 6 6 4;16 16 18 18];   % 障碍物
Q6=[16 18 18 16;4 4 6 6];   % 障碍物
Q7=[12 14 14 12;2 2 4 4];   % 障碍物
Q8=[6 8 8 6;12 12 14 14];   % 障碍物
% 障碍物列表
QLIST = [Q1(1,1:4),Q1(2,1:4);
         Q2(1,1:4),Q2(2,1:4);
         Q3(1,1:4),Q3(2,1:4);
         Q4(1,1:4),Q4(2,1:4);
         Q5(1,1:4),Q5(2,1:4);
         Q6(1,1:4),Q6(2,1:4);
         Q7(1,1:4),Q7(2,1:4);
         Q8(1,1:4),Q8(2,1:4);];
% QLIST = [];
% QLIST = [Q2(1,1:4),Q2(2,1:4);Q3(1,1:4),Q3(2,1:4)];
% 设置地图范围
MAX_X = 20;
MAX_Y = 20;
% 绘制地图，起始点和终止点
figure(1)
set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [500, 50, 700, 700]);
set(gca, 'XTick', 0:1:MAX_X);
set(gca, 'YTick', 0:1:MAX_Y);
grid on;
axis equal;        
axis ([0 MAX_X 0 MAX_Y ]);
hold on;
LL = 0.3;
fill([qset(1,1)-LL,qset(1,1)+LL,qset(1,1)+LL,qset(1,1)-LL],[qset(2,1)-LL,qset(2,1)-LL,qset(2,1)+LL,qset(2,1)+LL],'r'); % 起始点
fill([qset(1,2)-LL,qset(1,2)+LL,qset(1,2)+LL,qset(1,2)-LL],[qset(2,2)-LL,qset(2,2)-LL,qset(2,2)+LL,qset(2,2)+LL],'b'); % 终点
% 创建随机路图
[V,G,Index_startInPRM,Index_endInPRM]=build_PRM(n,k,QLIST,MAX_X,MAX_Y,qset);

%% 1. 利用A star算法寻路
path = A_star_search( G, V, Index_startInPRM,Index_endInPRM );
repath = [];
repath = [repath;qset(:,1)'];
kk = size(path,1);
for i=1:size(path,1)
    repath = [repath;path(kk,:)];
    kk = kk-1;
end
repath = [repath;qset(:,2)'];
plot(repath(:,1)',repath(:,2)','m-','LineWidth',2)