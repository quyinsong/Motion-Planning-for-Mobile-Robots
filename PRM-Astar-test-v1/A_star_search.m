%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
%%----------------------------------------------
function path = A_star_search( graph, Vertex,Index_start,Index_end )
%A_STAR_SEARCH �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

% �ڵ��ܸ���
N = size(graph,2);
% ����ڵ�����ݽṹ
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
% �������������ľ�����㹫ʽ��DISTANCE_FLAG=0�������پ��룬DISTANCE_FLAG=1��ŷʽ����
DISTANCE_FLAG = 1;
% ������ʼ��
xStart = Vertex(1,Index_start);
yStart = Vertex(2,Index_start);
% �����յ�
xTarget = Vertex(1,Index_end);
yTarget = Vertex(2,Index_end);
% ��ʼ�����ڵ�
xFatherNode=xStart;
yFatherNode=yStart;
goal_cost= DISTANCE(xFatherNode,yFatherNode,xTarget,yTarget,DISTANCE_FLAG);
path_cost=0;
total_cost = goal_cost+path_cost;
Node_Father = [Index_start,Index_start,goal_cost,path_cost,total_cost];
% �����ڵ����CLOSED��
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,:)= Node_Father;
% ��������·��
while(1) %you have to dicide the Conditions for while loop exit 

    %
    %finish the while loop
    %
    % ��ȡ��ǰ���ڵ�����
    xFatherNode = Vertex(1,Node_Father(1)); yFatherNode = Vertex(2,Node_Father(1));
    % �жϵ�ǰ���ڵ�������ӽڵ����
    Node_Child_Num = 0;
    Index_Child = [];
    for i=1:N
        if(graph(Node_Father(1),i)==1)
            Node_Child_Num = Node_Child_Num+1;
            Index_Child(Node_Child_Num) = i;
        end
    end 
    % ��ʼ���ӽڵ�List
    Node_Child_List = repmat(Node,Node_Child_Num,1);
    % ����ǰ���ڵ�������ӽڵ�ѹ��Node_Child_List������ѭ������ʼ��Ϊ���ڵ㣩
    for i=1:Node_Child_Num
        Node_Child_List(i,1) = Index_Child(i);
    end
    % ����Node_Child_Num���ӽڵ㣬�������Ҫ����ӽڵ���Ϣ,�����������OPEN��
    for i=1:Node_Child_Num
        % ��ǰ�ӽڵ�����
        xChildNode = Vertex(1,Node_Child_List(i,1));
        yChildNode = Vertex(2,Node_Child_List(i,1));
        % ����ӽڵ���CLOSED�У�������Ҫ��
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
        % ��������Ҫ����ӽڵ���Ϣ
        goal_cost=distance(xChildNode,yChildNode,xTarget,yTarget);
        path_cost = DISTANCE(xFatherNode,yFatherNode,xChildNode,yChildNode,DISTANCE_FLAG);
        total_cost = path_cost+goal_cost;
        Node_Child_List(i,2) = Node_Father(1);
        Node_Child_List(i,3) = goal_cost;
        Node_Child_List(i,4) = path_cost;
        Node_Child_List(i,5) = total_cost;
        % �жϵ�ǰ�ڵ��Ƿ���OPEN��
        CHILDE_IS_IN_OPEN_FLAG = 0;
        INDEX_CHILD_IN_OPEN = 0;
        for j=1:OPEN_COUNT
            if(OPEN(j,1)==Node_Child_List(i,1))
                CHILDE_IS_IN_OPEN_FLAG = 1;
                INDEX_CHILD_IN_OPEN = j;
                break;
            end
        end
        % �������OPEN�м��룬�������OPEN�����жϵ�ǰ·��total_cost�Ƿ��֮ǰ��С��
        % �����С�������֮ǰ�ڵ���OPEN�е���Ϣ
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
    % �ж�OPEN���Ƿ��Ѿ����ֵ�Ŀ��ڵ�
    TARGET_IS_IN_OPEN_FLAG = 0;
    for j=1:OPEN_COUNT
        if(OPEN(j,1)== Index_end)
            TARGET_IS_IN_OPEN_FLAG=1;
            % ��Ŀ��ڵ����CLOSED��
            CLOSED_COUNT = CLOSED_COUNT+1;
            CLOSED(CLOSED_COUNT,:) = OPEN(j,:);
        end
    end
    % ����ҵ�Ŀ��ڵ㣬����ֹwhileѭ��
    if (TARGET_IS_IN_OPEN_FLAG==1)
        break;
    end
    % �ж�OPEN�Ƿ�Ϊ�գ�OPENΪ�������û�ҵ�·��
    IS_OPEN_EMPTY_FLAG = false;
    if OPEN_COUNT == 0
        IS_OPEN_EMPTY_FLAG = true;
    end
    if IS_OPEN_EMPTY_FLAG
        break;
    end
    % ���û�ҵ����ҳ���ǰ������С�Ľڵ�
    temp_cost=OPEN(1,5);
    INDEX_MINCOST_IN_OPEN = 1;
    for j=2:OPEN_COUNT
        if (temp_cost>OPEN(j,5))
            temp_cost = OPEN(j,5);
            INDEX_MINCOST_IN_OPEN = j;
        end
    end
    % ������Ϊ���ڵ�
    Node_Father = OPEN(INDEX_MINCOST_IN_OPEN,:);
    % �������CLOSED
    CLOSED_COUNT = CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,:) = OPEN(INDEX_MINCOST_IN_OPEN,:);
    % ���䵯��OPEN��������OPEN
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
%% ѭ����ɺ�����
if TARGET_IS_IN_OPEN_FLAG %OpenList���������Ŀ����������������ҵ�·��,����CloseList�е����ݻ���ͼ��
    PATH = [xTarget yTarget];
    Index = Index_end;
    while 1        
        for i = 1:CLOSED_COUNT % Ѱ�ҵ�ǰ�ڵ�ĸ��ڵ�
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
elseif IS_OPEN_EMPTY_FLAG %OpenListΪ�������û�ҵ�·��
    msgbox('δ������·����');
end

path = PATH;
end


% ���������ľ�����㣬�ɸ���flagѡ��ŷ�Ͼ�����������پ���
function d = DISTANCE(xTarget,yTarget,xGoal,yGoal,flag)
    % flag==0�������پ��룬flag==1��ŷ�Ͼ���
    if (flag==0)
        d = abs(xTarget-xGoal)+abs(yTarget-yGoal);
    elseif(flag==1)
        d = distance(xTarget,yTarget,xGoal,yGoal);
    end
end
% ŷʽ�������
function dist = distance(x1,y1,x2,y2)
%This function calculates the distance between any two cartesian 
%coordinates.
%   Copyright 2009-2010 The MathWorks, Inc.
dist=sqrt((x1-x2)^2 + (y1-y2)^2);
end





















