%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
%%----------------------------------------------
function [Index_startInPRM, Index_endInPRM] = findStartAndEndInPRM( Vertex, qset, QLIST)
%FINDSTART 此处显示有关此函数的摘要
% 障碍物个数
Obstacle_Num = size(QLIST,1);
%   此处显示详细说明
V = Vertex;
%% 找到起始点和终止点的k个邻近点
k=5;
Index_AllNearStart= knnsearch( V', qset(:,1)', 'k',k);
Index_AllNearEnd = knnsearch(V',qset(:,2)','k',k);
for i=1:k
    plot(V(1,Index_AllNearStart(i)),V(2,Index_AllNearStart(i)),'ro'); hold on;
    plot(V(1,Index_AllNearEnd(i)),V(2,Index_AllNearEnd(i)),'bo'); hold on;
end
%% 从k个邻近点中找到满足要求的起始点
IS_FIND_START_FLAG=0;
IS_FIND_END_FLAG=0;
if Obstacle_Num==0
    % 由于没有障碍物，所以一定存在最近点
    Index_startInPRM = Index_AllNearStart(1);
else
    for i=1:k   % this loop is find nearest nodes to qset
        % 检测起始点和邻近点的连线是否经过障碍物
        for m=1:Obstacle_Num
            Q = [QLIST(m,1:4);QLIST(m,5:8)];
            if isequal( 1, isIntersection( V(:,Index_AllNearStart(i)),qset(:,1),Q ) )
                break;
            elseif m==Obstacle_Num
                % 如果不经过障碍物直接返回当前起始点邻近节点的索引值
                Index_startInPRM = Index_AllNearStart(i);
                IS_FIND_START_FLAG = 1;
                plot(V(1,Index_startInPRM),V(2,Index_startInPRM),'b*'); hold on;
            end
        end  
        % 如果找到符合条件的邻近起点，直接break
        if IS_FIND_START_FLAG
            break;
        end
        % 未找到则给出提示
        if i==k && IS_FIND_START_FLAG==0
            msgbox('未搜索到起始点！');
        end
    end
end
%% 从k个邻近点中找到满足要求的终止点
if Obstacle_Num==0
    % 由于没有障碍物，所以一定存在最近点
    Index_endInPRM = Index_AllNearEnd(1);
else
    for i=1:k   % this loop is find nearest nodes to qset
        % 检测起始点和邻近点的连线是否经过障碍物
        for m=1:Obstacle_Num
            Q = [QLIST(m,1:4);QLIST(m,5:8)];
            if isequal( 1, isIntersection( V(:,Index_AllNearEnd(i)),qset(:,2),Q ) )
                break;
            elseif m==Obstacle_Num
                % 如果不经过障碍物直接返回当前起始点邻近节点的索引值
                Index_endInPRM = Index_AllNearEnd(i);
                IS_FIND_END_FLAG = 1;
                plot(V(1,Index_endInPRM),V(2,Index_endInPRM),'b*'); hold on;
            end
        end  
        % 如果找到符合条件的邻近起点，直接break
        if IS_FIND_END_FLAG
            break;
        end  
        % 未找到则给出提示
        if i==k && IS_FIND_END_FLAG==0
            msgbox('未搜索到终点！');
        end
    end
end

end

