%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
% Update: Add the start and end point (qset) into this function
%%----------------------------------------------
function [allV,graph,Index_startInPRM,Index_endInPRM]= build_PRM(n,k,QLIST,MAX_X,MAX_Y,qset)
% 随机生成节点数组
V=[]; 
% 初始化图的二维数组（n+2 * n+2）----包含起始点和终止点qset
G=zeros(n+2); 
% 障碍物个数
Obstacle_Num = size(QLIST,1);
%% 绘制障碍物模型
for i=1:Obstacle_Num
    Q = [QLIST(i,1:4);QLIST(i,5:8)];
    fill(Q(1,:),Q(2,:),'k');
    hold on
end
%% 初始化图G的相邻节点间边的权值
for i=1:n+2
    for j=1:n+2
        if ~isequal(i,j)
         G(i,j)=inf;
        end
    end
end
%% 随机生成n个节点，判断是否落在障碍物上，如果不在障碍物中，将其加入节点数组V中
% 将初始点和终止点先加入V中
V = [V,qset(:,1)];
V = [V,qset(:,2)];
% 返回搜索初始点和终止点的索引
Index_startInPRM = 1;
Index_endInPRM = 2;
% 随机生成节点加入V中
if Obstacle_Num==0
    while size(V,2)<n+2
        qrand=[MAX_X*rand;MAX_Y*rand];
        V=[V,qrand]; % 由于没有障碍物，直接填充即可
    end
else
    while size(V,2)<n+2
        qrand=[MAX_X*rand;MAX_Y*rand];
        % 存在障碍物时，需要剔除随机生成在障碍物上的点，保留符合条件的点
        for m=1:Obstacle_Num
            Q = [QLIST(m,1:4);QLIST(m,5:8)];
            Q = [Q,Q(:,1)];
            [in,on] = inpolygon(qrand(1),qrand(2),Q(1,:),Q(2,:));
            if (in==1 || on==1)
                plot(qrand(1),qrand(2),'r*'); hold on;
                break;
            elseif m==Obstacle_Num
                V=[V,qrand];
                plot(qrand(1),qrand(2),'k*'); hold on;
            end
        end
    end
end
%% 根据上面生成的n个节点，填充图G的数据
if Obstacle_Num==0
    for j=1:size(V,2)
        % 找出当前节点j邻近的k个节点，找出的邻近点包括自己，且为第一个索引值，即ind(1)
        ind= knnsearch( V', V(:,j)', 'k',k+1);
        ind_temp = [];
        for i=2:size(ind,2)
            % 保证邻近节点Nq不包括自己
            ind_temp = [ind_temp,ind(i)];
            Nq(:,i-1)= V(:,ind(i));
        end
        ind = ind_temp;
        % 遍历这k个节点，因为没有障碍物，所以直接把当前节点与其邻近节点相连即可
        for i=1:size(Nq,2)
            G(j,ind(i))=1; % 如果一直到最后一个障碍物都没有交叉，则此邻近节点与j节点可通行，权值设为1
            G(ind(i),j)=1;
            plot([V(1,j),Nq(1,i)],[V(2,j),Nq(2,i)],'g--');
            pause(0.01);
            hold on
        end        
    end % 结束 for 循环
else
    for j=1:size(V,2)
        plot(V(1,j),V(2,j),'rs'); hold on;
        % 找出当前节点j邻近的k个节点，找出的邻近点包括自己，且为第一个索引值，即ind(1)
        ind= knnsearch( V', V(:,j)', 'k',k+1);
        ind_temp = [];
        % 保证邻近节点Nq不包括自己
        for i=2:size(ind,2)
            ind_temp = [ind_temp,ind(i)];
            Nq(:,i-1)= V(:,ind(i));
            plot(Nq(1,i-1),Nq(2,i-1),'bs'); hold on;
        end
        ind = ind_temp;
        % 遍历这k个节点，如果与父节点连线不与障碍物碰撞，则G中相应权值为1，代表两节点之间可通行
        for i=1:size(Nq,2)
            % 遍历所有障碍物，判断第i个邻近节点与j节点的连线与障碍物是否有交叉
            for m=1:Obstacle_Num
                Q = [QLIST(m,1:4);QLIST(m,5:8)];
                if  isIntersection( V(:,j),Nq(:,i),Q )==1
%                     plot([V(1,j),Nq(1,i)],[V(2,j),Nq(2,i)],'r--');
                    break; % 如果有交叉，直接break，进入下一个邻近节点的判断
                elseif m==Obstacle_Num
                    G(j,ind(i))=1; % 如果一直到最后一个障碍物都没有交叉，则此邻近节点与j节点可通行，权值设为1
                    G(ind(i),j)=1;
                    plot([V(1,j),Nq(1,i)],[V(2,j),Nq(2,i)],'g--');
%                     pause(0.01);
                    hold on
                end
            end
        end            
    end % 结束 for 循环
end % 结束 if和else 判断

% 输出图G和节点信息V
allV=V;
graph=G;

end
