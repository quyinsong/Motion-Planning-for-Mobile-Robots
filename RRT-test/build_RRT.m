%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
% reference: https://github.com/gn7550/RRT-PRM-codes
%%----------------------------------------------
function [GEd,qw]=build_RRT(qset, NumNodes, dq, QLIST)
qinit=qset(:,1); qgoal=qset(:,2);
G=qinit; GE=[];
% 障碍物个数
Obstacle_Num = size(QLIST,1);
%% 绘制障碍物模型
for i=1:Obstacle_Num
    Q = [QLIST(i,1:4);QLIST(i,5:8)];
    fill(Q(1,:),Q(2,:),'k');
    hold on
end

% Gph=zeros(n);
% for i=1:NumNodes
%     for j=1:NumNodes
%         if i~=j
%             Gph(i,j)=inf;
%         end
%     end
% end
fig_num=1;
for k= 1:NumNodes
    qrand=[rand*20; rand*20];
    C(:,k)=qrand;
    qnear= Nearest_Vertex( C(:,k) , G);
    qnew= qnear + dq*(C(:,k)-qnear)/norm(C(:,k)-qnear);
    if Obstacle_Num==0
        G=[G,qnew];
        GE= [GE, [qnear;qnew]];
        plot([qnear(1),qnew(1)],[qnear(2),qnew(2)])
        pause(0.1);
    else
        for i=1:Obstacle_Num
            Q = [QLIST(i,1:4);QLIST(i,5:8)];
            if isequal(1,isIntersection( qnear,qnew,Q ) )
                break;
            elseif i==Obstacle_Num
                % 如果到最后一个障碍物都不碰撞，则将新节点及其父节点保存进GE
                G=[G,qnew];
                GE= [GE, [qnear;qnew]];
                % 保存gif，与算法无关
                plot([qnear(1),qnew(1)],[qnear(2),qnew(2)])
                pause(0.1);
                hold on
                F=getframe(gcf);
                I=frame2im(F);
                [I,map]=rgb2ind(I,256);
                if (fig_num==1)
                    imwrite(I,map,'test.gif','gif','Loopcount',inf,'DelayTime',0.2);
                elseif (rem(fig_num,10)==0)
                    imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0.2);
                end
                fig_num = fig_num+1;
            end
        end
    end
    
    if isequal(qnew,qgoal) || norm(qnew-qgoal)<dq
        GE= [GE, [qnew;qgoal]];
        plot([qnear(1),qnew(1)],[qnear(2),qnew(2)])
        hold on

        break
    end
end

qw=G;
GEd=GE;

end

function vv=Nearest_Vertex(q,G)
d= inf;
n= size(G,2);
for i=1:n
    if norm(G(:,i) - q) < d && ~isequal(G(:,i),q)
        vnew= G(:,i);
        d= norm(G(:,i)-q);
    end
end
vv= vnew;
end