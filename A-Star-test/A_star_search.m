function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    TARGET = [xTarget,yTarget];
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    START = [xStart,yStart];
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % ����ڵ�����ݽṹ
    %--------------------------------------------------------------------------
    %Node |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    Node = zeros(1,7);   
    % OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    % |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=repmat(Node,MAX_X*MAX_Y,1);
    OPEN_COUNT=0;
    % CLOSED LIST STRUCTURE
    %--------------------------------------------------------------------------
    % |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    CLOSED=repmat(Node,MAX_X*MAX_Y,1);
    CLOSED_COUNT=0;
    % �������������ľ�����㹫ʽ��DISTANCE_FLAG=0�������پ��룬DISTANCE_FLAG=1��ŷʽ����
    DISTANCE_FLAG = 1;
    % ��ʼ�����ڵ�
    xFatherNode=xStart;
    yFatherNode=yStart;
    goal_cost= DISTANCE(xFatherNode,yFatherNode,xTarget,yTarget,DISTANCE_FLAG);
    path_cost=0;
    total_cost = goal_cost+path_cost;
    Node_Father = [xFatherNode,yFatherNode,xFatherNode,yFatherNode,goal_cost,path_cost,total_cost];
    % �����ڵ����CLOSED��
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,:)= Node_Father;
%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fig_num = 1;
    while(1) %you have to dicide the Conditions for while loop exit 
     
        %
        %finish the while loop
        %
        % ��ȡ��ǰ���ڵ�����
        xFatherNode = Node_Father(1); yFatherNode = Node_Father(2);
        % ��ʼ���ӽڵ�List
        Node_Child_List = repmat(Node,8,1);
        % ���ҵ�ǰ���ڵ�������ӽڵ㣨����ѭ���ĸ��ڵ�Ϊ��ʼ�㣩
        Node_Child_List(1,1) = xFatherNode;   Node_Child_List(1,2) = yFatherNode+1;
        Node_Child_List(2,1) = xFatherNode+1; Node_Child_List(2,2) = yFatherNode+1;
        Node_Child_List(3,1) = xFatherNode+1; Node_Child_List(3,2) = yFatherNode;
        Node_Child_List(4,1) = xFatherNode+1; Node_Child_List(4,2) = yFatherNode-1;
        Node_Child_List(5,1) = xFatherNode;   Node_Child_List(5,2) = yFatherNode-1;
        Node_Child_List(6,1) = xFatherNode-1; Node_Child_List(6,2) = yFatherNode-1;
        Node_Child_List(7,1) = xFatherNode-1; Node_Child_List(7,2) = yFatherNode;
        Node_Child_List(8,1) = xFatherNode-1; Node_Child_List(8,2) = yFatherNode+1;
        % �����˸��ӽڵ㣬�������Ҫ����ӽڵ���Ϣ,�����������OPEN��
        for i=1:8
            % ��ǰ�ӽڵ�����
            xChildNode = Node_Child_List(i,1);
            yChildNode = Node_Child_List(i,2);
            % ����ӽڵ㳬����ͼ��Χ��������Ҫ��
            if(xChildNode<1 || xChildNode>MAX_X || yChildNode<1 || yChildNode>MAX_Y)
               continue;
            end
            % ����ӽڵ�λ���ϰ����У�������Ҫ��
            if(MAP(xChildNode,yChildNode)==-1)
                continue;
            end
            % ����ӽڵ���CLOSED�У�������Ҫ��
            CHILD_IS_IN_CLOSED = 0;
            for j=1:CLOSED_COUNT
                if(CLOSED(j,1)==xChildNode && CLOSED(j,2)==yChildNode)
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
            Node_Child_List(i,1) = 0;
            Node_Child_List(i,4) = xFatherNode;
            Node_Child_List(i,5) = yFatherNode;
            Node_Child_List(i,6) = goal_cost;
            Node_Child_List(i,7) = path_cost;
            Node_Child_List(i,8) = total_cost;
            % �жϵ�ǰ�ڵ��Ƿ���OPEN��
            CHILDE_IS_IN_OPEN_FLAG = 0;
            INDEX_CHILD_IN_OPEN = 0;
            for j=1:OPEN_COUNT
                if(OPEN(j,1)==xChildNode && OPEN(j,2)==yChildNode)
                    CHILDE_IS_IN_OPEN_FLAG = 1;
                    INDEX_CHILD_IN_OPEN = j;
                    break;
                end
            end
            % �������OPEN�м��룬�������OPEN�����жϵ�ǰ·��total_cost�Ƿ��֮ǰ��С��
            % �����С�������֮ǰ�ڵ���OPEN�е���Ϣ
            if (CHILDE_IS_IN_OPEN_FLAG)
                if (total_cost<OPEN(INDEX_CHILD_IN_OPEN,7))
                    OPEN(INDEX_CHILD_IN_OPEN,3) = xFatherNode;
                    OPEN(INDEX_CHILD_IN_OPEN,4) = yFatherNode;
                    OPEN(INDEX_CHILD_IN_OPEN,6) = path_cost;
                    OPEN(INDEX_CHILD_IN_OPEN,7) = total_cost;
                end
            else
                OPEN_COUNT = OPEN_COUNT+1;
                OPEN(OPEN_COUNT,1) = xChildNode;
                OPEN(OPEN_COUNT,2) = yChildNode;
                OPEN(OPEN_COUNT,3) = xFatherNode;
                OPEN(OPEN_COUNT,4) = yFatherNode;
                OPEN(OPEN_COUNT,5) = goal_cost;
                OPEN(OPEN_COUNT,6) = path_cost;
                OPEN(OPEN_COUNT,7) = total_cost;
            end
        end
        % �ж�OPEN���Ƿ��Ѿ����ֵ�Ŀ��ڵ�
        TARGET_IS_IN_OPEN_FLAG = 0;
        for j=1:OPEN_COUNT
            if(OPEN(j,1)==xTarget && OPEN(j,2)==yTarget)
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
        temp_cost=OPEN(1,7);
        INDEX_MINCOST_IN_OPEN = 1;
        for j=2:OPEN_COUNT
            if (temp_cost>OPEN(j,7))
                temp_cost = OPEN(j,7);
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
        
        % ���ӻ������б�OPEN��CLOSED
        visualize_LIST( CLOSED,CLOSED_COUNT,OPEN,OPEN_COUNT,START,TARGET );
        pause(0.5);
        F=getframe(gcf);
        I=frame2im(F);
        [I,map]=rgb2ind(I,256);
        if (fig_num==1)
            imwrite(I,map,'test.gif','gif','Loopcount',inf,'DelayTime',0.2);
        else
            imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0.2);
        end
        fig_num = fig_num+1;
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
        PositionX = xTarget;
        PositionY = yTarget;
        PATH = [xTarget yTarget];
        while 1        
            for i = 1:CLOSED_COUNT % Ѱ�ҵ�ǰ�ڵ�ĸ��ڵ�
                if PositionX == CLOSED(i,1) && PositionY == CLOSED(i,2)
                    PositionX = CLOSED(i,3);
                    PositionY = CLOSED(i,4);      
                    Pos = [PositionX PositionY];
                    PATH = [PATH;Pos];
                    break;
                end
            end      
            if PositionX == xStart && PositionY == yStart
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
