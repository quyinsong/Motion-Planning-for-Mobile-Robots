%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
%%----------------------------------------------
function [Index_startInPRM, Index_endInPRM] = findStartAndEndInPRM( Vertex, qset, QLIST)
%FINDSTART �˴���ʾ�йش˺�����ժҪ
% �ϰ������
Obstacle_Num = size(QLIST,1);
%   �˴���ʾ��ϸ˵��
V = Vertex;
%% �ҵ���ʼ�����ֹ���k���ڽ���
k=5;
Index_AllNearStart= knnsearch( V', qset(:,1)', 'k',k);
Index_AllNearEnd = knnsearch(V',qset(:,2)','k',k);
for i=1:k
    plot(V(1,Index_AllNearStart(i)),V(2,Index_AllNearStart(i)),'ro'); hold on;
    plot(V(1,Index_AllNearEnd(i)),V(2,Index_AllNearEnd(i)),'bo'); hold on;
end
%% ��k���ڽ������ҵ�����Ҫ�����ʼ��
IS_FIND_START_FLAG=0;
IS_FIND_END_FLAG=0;
if Obstacle_Num==0
    % ����û���ϰ������һ�����������
    Index_startInPRM = Index_AllNearStart(1);
else
    for i=1:k   % this loop is find nearest nodes to qset
        % �����ʼ����ڽ���������Ƿ񾭹��ϰ���
        for m=1:Obstacle_Num
            Q = [QLIST(m,1:4);QLIST(m,5:8)];
            if isequal( 1, isIntersection( V(:,Index_AllNearStart(i)),qset(:,1),Q ) )
                break;
            elseif m==Obstacle_Num
                % ����������ϰ���ֱ�ӷ��ص�ǰ��ʼ���ڽ��ڵ������ֵ
                Index_startInPRM = Index_AllNearStart(i);
                IS_FIND_START_FLAG = 1;
                plot(V(1,Index_startInPRM),V(2,Index_startInPRM),'b*'); hold on;
            end
        end  
        % ����ҵ������������ڽ���㣬ֱ��break
        if IS_FIND_START_FLAG
            break;
        end
        % δ�ҵ��������ʾ
        if i==k && IS_FIND_START_FLAG==0
            msgbox('δ��������ʼ�㣡');
        end
    end
end
%% ��k���ڽ������ҵ�����Ҫ�����ֹ��
if Obstacle_Num==0
    % ����û���ϰ������һ�����������
    Index_endInPRM = Index_AllNearEnd(1);
else
    for i=1:k   % this loop is find nearest nodes to qset
        % �����ʼ����ڽ���������Ƿ񾭹��ϰ���
        for m=1:Obstacle_Num
            Q = [QLIST(m,1:4);QLIST(m,5:8)];
            if isequal( 1, isIntersection( V(:,Index_AllNearEnd(i)),qset(:,2),Q ) )
                break;
            elseif m==Obstacle_Num
                % ����������ϰ���ֱ�ӷ��ص�ǰ��ʼ���ڽ��ڵ������ֵ
                Index_endInPRM = Index_AllNearEnd(i);
                IS_FIND_END_FLAG = 1;
                plot(V(1,Index_endInPRM),V(2,Index_endInPRM),'b*'); hold on;
            end
        end  
        % ����ҵ������������ڽ���㣬ֱ��break
        if IS_FIND_END_FLAG
            break;
        end  
        % δ�ҵ��������ʾ
        if i==k && IS_FIND_END_FLAG==0
            msgbox('δ�������յ㣡');
        end
    end
end

end

