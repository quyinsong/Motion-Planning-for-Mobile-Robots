%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
%%----------------------------------------------
function path = A_star_search( graph, Vertex,Index_start,Index_end )
%A_STAR_SEARCH 此处显示有关此函数的摘要
%   此处显示详细说明

% 节点总个数
N = size(graph,2);
% 定义节点的数据结构
%--------------------------------------------------------------------------
% Node |Index |Parent Index |h(n) |g(n) |f(n)|
%--------------------------------------------------------------------------
Node = zeros(1,5);
% OPEN LIST STRUCTURE
%--------------------------------------------------------------------------
% |Index |Parent Index |h(n) |g(n) |f(n)|
%--------------------------------------------------------------------------
OPEN=repmat(Node,N*N,1);
OPEN_COUNT=0;
% CLOSED LIST STRUCTURE
%--------------------------------------------------------------------------
% |Index |Parent Index |h(n) |g(n) |f(n)|
%--------------------------------------------------------------------------
CLOSED=repmat(Node,N*N,1);
CLOSED_COUNT=0;
% 设置启发函数的距离计算公式，DISTANCE_FLAG=0：曼哈顿距离，DISTANCE_FLAG=1：欧式距离
DISTANCE_FLAG = 1;
% 搜索起始点
xStart = Vertex(1,Index_start);
yStart = Vertex(2,Index_start);
% 搜索终点
xTarget = Vertex(1,Index_end);
yTarget = Vertex(2,Index_end);
% 初始化父节点
xFatherNode=xStart;
yFatherNode=yStart;
goal_cost= DISTANCE(xFatherNode,yFatherNode,xTarget,yTarget,DISTANCE_FLAG);
path_cost=0;
total_cost = goal_cost+path_cost;
Node_Father = [Index_start,Index_start,goal_cost,path_cost,total_cost];
% 将父节点加入CLOSED中
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,:)= Node_Father;
% 搜索最优路径
while(1) %you have to dicide the Conditions for while loop exit 

    %
    %finish the while loop
    %
    % 获取当前父节点坐标
    xFatherNode = Vertex(1,Node_Father(1)); yFatherNode = Vertex(2,Node_Father(1));
    % 判断当前父节点的所有子节点个数
    Node_Child_Num = 0;
    Index_Child = [];
    for i=1:N
        if(graph(Node_Father(1),i)==1)
            Node_Child_Num = Node_Child_Num+1;
            Index_Child(Node_Child_Num) = i;
        end
    end 
    % 初始化子节点List
    Node_Child_List = repmat(Node,Node_Child_Num,1);
    % 将当前父节点的所有子节点压入Node_Child_List（初次循环的起始点为父节点）
    for i=1:Node_Child_Num
        Node_Child_List(i,1) = Index_Child(i);
    end
    % 遍历Node_Child_Num个子节点，填充满足要求的子节点信息,并将其填充至OPEN中
    for i=1:Node_Child_Num
        % 当前子节点坐标
        xChildNode = Vertex(1,Node_Child_List(i,1));
        yChildNode = Vertex(2,Node_Child_List(i,1));
        % 如果子节点在CLOSED中，则不满足要求
        CHILD_IS_IN_CLOSED = 0;
        for j=1:CLOSED_COUNT
            if(CLOSED(j,1)==Node_Child_List(i,1))
                CHILD_IS_IN_CLOSED = 1;
                break;
            end
        end
        if (CHILD_IS_IN_CLOSED == 1)
            continue;
        end
        % 计算满足要求的子节点信息
        goal_cost=distance(xChildNode,yChildNode,xTarget,yTarget);
        path_cost = DISTANCE(xFatherNode,yFatherNode,xChildNode,yChildNode,DISTANCE_FLAG);
        total_cost = path_cost+goal_cost;
        Node_Child_List(i,2) = Node_Father(1);
        Node_Child_List(i,3) = goal_cost;
        Node_Child_List(i,4) = path_cost;
        Node_Child_List(i,5) = total_cost;
        % 判断当前节点是否在OPEN中
        CHILDE_IS_IN_OPEN_FLAG = 0;
        INDEX_CHILD_IN_OPEN = 0;
        for j=1:OPEN_COUNT
            if(OPEN(j,1)==Node_Child_List(i,1))
                CHILDE_IS_IN_OPEN_FLAG = 1;
                INDEX_CHILD_IN_OPEN = j;
                break;
            end
        end
        % 如果不在OPEN中加入，如果不在OPEN中则判断当前路径total_cost是否比之前的小，
        % 如果更小，则更新之前节点在OPEN中的信息
        if (CHILDE_IS_IN_OPEN_FLAG)
            if (total_cost<OPEN(INDEX_CHILD_IN_OPEN,5))
                OPEN(INDEX_CHILD_IN_OPEN,2) = Node_Father(1);
                OPEN(INDEX_CHILD_IN_OPEN,4) = path_cost;
                OPEN(INDEX_CHILD_IN_OPEN,5) = total_cost;
            end
        else
            OPEN_COUNT = OPEN_COUNT+1;
            OPEN(OPEN_COUNT,1) = Node_Child_List(i,1);
            OPEN(OPEN_COUNT,2) = Node_Father(1);
            OPEN(OPEN_COUNT,3) = goal_cost;
            OPEN(OPEN_COUNT,4) = path_cost;
            OPEN(OPEN_COUNT,5) = total_cost;
        end
    end
    % 判断OPEN中是否已经出现的目标节点
    TARGET_IS_IN_OPEN_FLAG = 0;
    for j=1:OPEN_COUNT
        if(OPEN(j,1)== Index_end)
            TARGET_IS_IN_OPEN_FLAG=1;
            % 将目标节点放入CLOSED中
            CLOSED_COUNT = CLOSED_COUNT+1;
            CLOSED(CLOSED_COUNT,:) = OPEN(j,:);
        end
    end
    % 如果找到目标节点，则终止while循环
    if (TARGET_IS_IN_OPEN_FLAG==1)
        break;
    end
    % 判断OPEN是否为空，OPEN为空则代表没找到路径
    IS_OPEN_EMPTY_FLAG = false;
    if OPEN_COUNT == 0
        IS_OPEN_EMPTY_FLAG = true;
    end
    if IS_OPEN_EMPTY_FLAG
        break;
    end
    % 如果没找到，找出当前代价最小的节点
    temp_cost=OPEN(1,5);
    INDEX_MINCOST_IN_OPEN = 1;
    for j=2:OPEN_COUNT
        if (temp_cost>OPEN(j,5))
            temp_cost = OPEN(j,5);
            INDEX_MINCOST_IN_OPEN = j;
        end
    end
    % 将其作为父节点
    Node_Father = OPEN(INDEX_MINCOST_IN_OPEN,:);
    % 将其加入CLOSED
    CLOSED_COUNT = CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,:) = OPEN(INDEX_MINCOST_IN_OPEN,:);
    % 将其弹出OPEN，并更新OPEN
    for j=INDEX_MINCOST_IN_OPEN:OPEN_COUNT
        OPEN(j,:) = OPEN(j+1,:);
    end
    OPEN_COUNT = OPEN_COUNT-1;

end %End of While Loop

%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

%
%How to get the optimal path after A_star search?
%please finish it
%
%% 循环完成后处理结果
if TARGET_IS_IN_OPEN_FLAG %OpenList中如果出现目标点则代表搜索完成找到路径,根据CloseList中的数据画出图像
    PATH = [xTarget yTarget];
    Index = Index_end;
    while 1        
        for i = 1:CLOSED_COUNT % 寻找当前节点的父节点
            if Index == CLOSED(i,1)
                Index = CLOSED(i,2);
                PositionX = Vertex(1,Index);
                PositionY = Vertex(2,Index);      
                Pos = [PositionX PositionY];
                PATH = [PATH;Pos];
                break;
            end
        end      
        if Index == Index_start
            break;
        end
    end
elseif IS_OPEN_EMPTY_FLAG %OpenList为空则代表没找到路径
    msgbox('未搜索到路径！');
end

path = PATH;
end


% 启发函数的距离计算，可根据flag选择欧氏距离或是曼哈顿距离
function d = DISTANCE(xTarget,yTarget,xGoal,yGoal,flag)
    % flag==0：曼哈顿距离，flag==1：欧氏距离
    if (flag==0)
        d = abs(xTarget-xGoal)+abs(yTarget-yGoal);
    elseif(flag==1)
        d = distance(xTarget,yTarget,xGoal,yGoal);
    end
end
% 欧式距离计算
function dist = distance(x1,y1,x2,y2)
%This function calculates the distance between any two cartesian 
%coordinates.
%   Copyright 2009-2010 The MathWorks, Inc.
dist=sqrt((x1-x2)^2 + (y1-y2)^2);
end





















