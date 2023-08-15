%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
% reference: https://github.com/gn7550/RRT-PRM-codes
%%----------------------------------------------
function [GEd,qw]=build_RRT(qset, NumNodes, dq, QLIST)
qinit=qset(:,1); qgoal=qset(:,2);
G=qinit; GE=[];
% ������1����������ָ���յ�ĵ�λ��������Ϊ������
lambda = 1.5; % ��������ռ�ı���
guide_vector1 = (qgoal-qinit)/norm(qgoal-qinit);
% ������2���������еڶ���������
% �ڶ�����������Ҫ����������½ڵ���ҵ��丸�ڵ�ʱ���㣬��guide_vector2
% �ϰ������
Obstacle_Num = size(QLIST,1);
%% �����ϰ���ģ��
for i=1:Obstacle_Num
    Q = [QLIST(i,1:4);QLIST(i,5:8)];
    fill(Q(1,:),Q(2,:),'k');
    hold on
end
% RRT
fig_num=1;
for k= 1:NumNodes
    qrand=[rand*20; rand*20];
    C(:,k)=qrand;
    qnear= Nearest_Vertex( C(:,k) , G);
    % ����������������ܵ��½ڵ�
    guide_vector2 = (qgoal-qnear)/norm(qgoal-qnear);
    forward_vector = lambda*guide_vector2+(C(:,k)-qnear)/norm(C(:,k)-qnear);
    qnew= qnear + dq*forward_vector/norm(forward_vector);
    % �ж��½ڵ��Ƿ����Ҫ��
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
                % ��������һ���ϰ��ﶼ����ײ�����½ڵ㼰�丸�ڵ㱣���GE
                G=[G,qnew];
                GE= [GE, [qnear;qnew]];
                % ����gif�����㷨�޹�
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