%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
%%----------------------------------------------
function [allV,graph]= build_PRM(n,k,QLIST,MAX_X,MAX_Y)
% ������ɽڵ�����
V=[]; 
% ��ʼ��ͼ�Ķ�ά���飨n * n��
G=zeros(n); 
% �ϰ������
Obstacle_Num = size(QLIST,1);
%% �����ϰ���ģ��
for i=1:Obstacle_Num
    Q = [QLIST(i,1:4);QLIST(i,5:8)];
    fill(Q(1,:),Q(2,:),'k');
    hold on
end
%% ��ʼ��ͼG�����ڽڵ��ߵ�Ȩֵ
for i=1:n
    for j=1:n
        if ~isequal(i,j)
         G(i,j)=inf;
        end
    end
end
%% �������n���ڵ㣬�ж��Ƿ������ϰ����ϣ���������ϰ����У��������ڵ�����V��
if Obstacle_Num==0
    while size(V,2)<n
        qrand=[MAX_X*rand;MAX_Y*rand];
        V=[V,qrand]; % ����û���ϰ��ֱ����伴��
    end
else
    while size(V,2)<n
        qrand=[MAX_X*rand;MAX_Y*rand];
        % �����ϰ���ʱ����Ҫ�޳�����������ϰ����ϵĵ㣬�������������ĵ�
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
%% �����������ɵ�n���ڵ㣬���ͼG������
if Obstacle_Num==0
    for j=1:size(V,2)
        % �ҳ���ǰ�ڵ�j�ڽ���k���ڵ㣬�ҳ����ڽ�������Լ�����Ϊ��һ������ֵ����ind(1)
        ind= knnsearch( V', V(:,j)', 'k',k+1);
        ind_temp = [];
        for i=2:size(ind,2)
            % ��֤�ڽ��ڵ�Nq�������Լ�
            ind_temp = [ind_temp,ind(i)];
            Nq(:,i-1)= V(:,ind(i));
        end
        ind = ind_temp;
        % ������k���ڵ㣬��Ϊû���ϰ������ֱ�Ӱѵ�ǰ�ڵ������ڽ��ڵ���������
        for i=1:size(Nq,2)
            G(j,ind(i))=1; % ���һֱ�����һ���ϰ��ﶼû�н��棬����ڽ��ڵ���j�ڵ��ͨ�У�Ȩֵ��Ϊ1
            G(ind(i),j)=1;
            plot([V(1,j),Nq(1,i)],[V(2,j),Nq(2,i)],'g--');
            pause(0.01);
            hold on
        end        
    end % ���� for ѭ��
else
    for j=1:size(V,2)
        plot(V(1,j),V(2,j),'rs'); hold on;
        % �ҳ���ǰ�ڵ�j�ڽ���k���ڵ㣬�ҳ����ڽ�������Լ�����Ϊ��һ������ֵ����ind(1)
        ind= knnsearch( V', V(:,j)', 'k',k+1);
        ind_temp = [];
        % ��֤�ڽ��ڵ�Nq�������Լ�
        for i=2:size(ind,2)
            ind_temp = [ind_temp,ind(i)];
            Nq(:,i-1)= V(:,ind(i));
            plot(Nq(1,i-1),Nq(2,i-1),'bs'); hold on;
        end
        ind = ind_temp;
        % ������k���ڵ㣬����븸�ڵ����߲����ϰ�����ײ����G����ӦȨֵΪ1���������ڵ�֮���ͨ��
        for i=1:size(Nq,2)
            % ���������ϰ���жϵ�i���ڽ��ڵ���j�ڵ���������ϰ����Ƿ��н���
            for m=1:Obstacle_Num
                Q = [QLIST(m,1:4);QLIST(m,5:8)];
                if  isIntersection( V(:,j),Nq(:,i),Q )==1
%                     plot([V(1,j),Nq(1,i)],[V(2,j),Nq(2,i)],'r--');
                    break; % ����н��棬ֱ��break��������һ���ڽ��ڵ���ж�
                elseif m==Obstacle_Num
                    G(j,ind(i))=1; % ���һֱ�����һ���ϰ��ﶼû�н��棬����ڽ��ڵ���j�ڵ��ͨ�У�Ȩֵ��Ϊ1
                    G(ind(i),j)=1;
                    plot([V(1,j),Nq(1,i)],[V(2,j),Nq(2,i)],'g--');
%                     pause(0.01);
                    hold on
                end
            end
        end            
    end % ���� for ѭ��
end % ���� if��else �ж�

% ���ͼG�ͽڵ���ϢV
allV=V;
graph=G;

end
