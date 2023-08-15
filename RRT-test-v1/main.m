%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
%%----------------------------------------------
%% Prob 1. Run RRT
close all
clear
clc
qset=[3 19;3 19]; % 1st column: q_initial, 2nd: q_goal
NumNodes=2000;
dq=1;
% ����α�ʾ���ϰ���
Q1=[1 2 2 1;1 1 2 2];   % �ϰ���
Q2=[10 15 15 10;
    10 10 15 15];   % �ϰ���
Q3=[5 10 10 5;
    5 5 10 10];   % �ϰ���
Q4=[2 4 4 2;14 14 16 16];   % �ϰ���
Q5=[4 6 6 4;16 16 18 18];   % �ϰ���
Q6=[16 18 18 16;4 4 6 6];   % �ϰ���
Q7=[12 14 14 12;2 2 4 4];   % �ϰ���
Q8=[6 8 8 6;12 12 14 14];   % �ϰ���
% �ϰ����б�
QLIST = [Q1(1,1:4),Q1(2,1:4);
         Q2(1,1:4),Q2(2,1:4);
         Q3(1,1:4),Q3(2,1:4);
         Q4(1,1:4),Q4(2,1:4);
         Q5(1,1:4),Q5(2,1:4);
         Q6(1,1:4),Q6(2,1:4);
         Q7(1,1:4),Q7(2,1:4);
         Q8(1,1:4),Q8(2,1:4);];

% ���õ�ͼ��Χ
MAX_X = 20;
MAX_Y = 20;
% ���Ƶ�ͼ����ʼ�����ֹ��
figure(1)
set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [500, 50, 700, 700]);
set(gca, 'XTick', 0:1:MAX_X);
set(gca, 'YTick', 0:1:MAX_Y);
grid on;
axis equal;        
axis ([0 MAX_X 0 MAX_Y ]);
hold on;
% ������ʼ����յ�
LL = 0.2;
fill([qset(1,1)-LL,qset(1,1)+LL,qset(1,1)+LL,qset(1,1)-LL],[qset(2,1)-LL,qset(2,1)-LL,qset(2,1)+LL,qset(2,1)+LL],'r'); % ��ʼ��
fill([qset(1,2)-LL,qset(1,2)+LL,qset(1,2)+LL,qset(1,2)-LL],[qset(2,2)-LL,qset(2,2)-LL,qset(2,2)+LL,qset(2,2)+LL],'b'); % �յ�
% ����RRT
[GE,qw]=build_RRT(qset, NumNodes, dq, QLIST);   %GE is [qnear ; qnew]
%the first and last columns have q_init and q_goal
qc1=GE(1:2,:); qc2=GE(3:4,:);
qc1=flip(qc1,2); % ���ڵ�
qc2=flip(qc2,2); % �ӽڵ�
qpath=[qc1(:,1)]; % Ŀ��ڵ�ĸ��ڵ�

while ~isequal(qpath(:,1),qset(:,1))
for i=1:size(GE,2)
    if isequal(qpath(:,1), qc2(:,i))
        qpath=[qc1(:,i) qpath];
        break;
    end
end
end
qpath= [qpath qset(:,2)];
disp(qpath)
plot(qpath(1,:),qpath(2,:),'b--o')
hold on