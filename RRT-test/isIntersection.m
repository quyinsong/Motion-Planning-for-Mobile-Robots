%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
%%----------------------------------------------
function IS_INTERSECT_FLAG = isIntersection( P0,P1,Q )
%ISINTERSECTION �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
% P0��P1����ʼ����յ�
% Q���ı���
% IS_INTERSECT_FLAG��Ϊ1���н��棬Ϊ0��û�н���
IS_INTERSECT_FLAG = 0;
% �ж�QΪ����
N = size(Q,2);
% ����Q�����������ز����ļ���
QQ = [Q,Q(:,1)]; 
% ��P0��P1�������x�����y����ֱ�����
x_P_sort = sort([P0(1),P1(1)]);
y_P_sort = sort([P0(2),P1(2)]);
% �ж�P0��P1�Ƿ���Q��
[P0_in,P0_on] = inpolygon(P0(1),P0(2),QQ(1,:),QQ(2,:));
[P1_in,P1_on] = inpolygon(P1(1),P1(2),QQ(1,:),QQ(2,:));
if (P0_in||P0_on) || (P1_in||P1_on)
    IS_INTERSECT_FLAG=1;
    disp('xxx1');
elseif (isequal(P0,P1))
    % ��������㶼����Q�У����������غϣ��򲻿��ܺ�Q�ཻ
    IS_INTERSECT_FLAG=0;
    disp('xxx2');
else
    % �����㲻�غ��Ҷ�����Q�У���������Q����
    % ����P0-P1���߲���
    if (P1(1)-P0(1)==0)
        k_P = inf;
        b_P = 0;
    else
        k_P = (P1(2)-P0(2))/(P1(1)-P0(1));
        b_P = P0(2)-k_P*P0(1);
    end
    % ѭ���������ж��Ƿ��н���
    for i=1:N
        % ����Q�ĵ�i���ߵĲ��� y = k_Q*x+b_Q
        if (QQ(1,i+1)-QQ(1,i)==0)
            k_Q(i) = inf;
            b_Q(i) = 0;
        else
            k_Q(i) = (QQ(2,i+1)-QQ(2,i))/(QQ(1,i+1)-QQ(1,i));
            b_Q(i) = QQ(2,i)-k_Q(i)*QQ(1,i);
        end
        % ���ƽ���򲻽��棬���������غ�
        if (k_Q(i)==k_P)
            % ����ĵ㹲�ߣ���Ȼ��Ϊ�ཻ
            if k_P==inf
                if (Q(1,i)==P0(1))
                    IS_INTERSECT_FLAG=1;
                    break;
                end
            else
                if (b_P==b_Q(i))
                    IS_INTERSECT_FLAG=1;
                    break;
                end
            end            
        else
            if k_P==inf
                % �����ƽ����һ�����棬���㽻��
                X_INTERSECT = P0(1);
                Y_INTERSECT = k_Q(i)*X_INTERSECT+b_Q(i);
                % �жϽ����Ƿ��������߶�֮��
                x_Q_sort = sort([QQ(1,i),QQ(1,i+1)]);
                y_Q_sort = sort([QQ(2,i),QQ(2,i+1)]);
                if (X_INTERSECT>=x_Q_sort(1)&&X_INTERSECT<=x_Q_sort(2)&&Y_INTERSECT>=y_Q_sort(1)&&Y_INTERSECT<=y_Q_sort(2))
                    if (X_INTERSECT>=x_P_sort(1)&&X_INTERSECT<=x_P_sort(2)&&Y_INTERSECT>=y_P_sort(1)&&Y_INTERSECT<=y_P_sort(2))
                        IS_INTERSECT_FLAG=1;
                        plot(X_INTERSECT,Y_INTERSECT,'c+');
                        disp('xxx3');
                        break;
                    end
                end
            elseif k_Q(i)==inf
                disp('xxx4');
                % �����ƽ����һ�����棬���㽻��
                X_INTERSECT = Q(1,i);
                Y_INTERSECT = k_P*X_INTERSECT+b_P;
                % �жϽ����Ƿ��������߶�֮��
                x_Q_sort = sort([QQ(1,i),QQ(1,i+1)]);
                y_Q_sort = sort([QQ(2,i),QQ(2,i+1)]);
                if (X_INTERSECT>=x_Q_sort(1)&&X_INTERSECT<=x_Q_sort(2)&&Y_INTERSECT>=y_Q_sort(1)&&Y_INTERSECT<=y_Q_sort(2))
                    if (X_INTERSECT>=x_P_sort(1)&&X_INTERSECT<=x_P_sort(2)&&Y_INTERSECT>=y_P_sort(1)&&Y_INTERSECT<=y_P_sort(2))
                        IS_INTERSECT_FLAG=1;
                        plot(X_INTERSECT,Y_INTERSECT,'c+');
                        break;
                    end
                end
            else
                % �����ƽ����һ�����棬���㽻��
                X_INTERSECT = (b_P-b_Q(i))/(k_Q(i)-k_P);
                Y_INTERSECT = k_Q(i)*X_INTERSECT+b_Q(i);
                % �жϽ����Ƿ��������߶�֮��
                x_Q_sort = sort([QQ(1,i),QQ(1,i+1)]);
                y_Q_sort = sort([QQ(2,i),QQ(2,i+1)]);
                if (X_INTERSECT>=x_Q_sort(1)&&X_INTERSECT<=x_Q_sort(2)&&Y_INTERSECT>=y_Q_sort(1)&&Y_INTERSECT<=y_Q_sort(2))
                    if (X_INTERSECT>=x_P_sort(1)&&X_INTERSECT<=x_P_sort(2)&&Y_INTERSECT>=y_P_sort(1)&&Y_INTERSECT<=y_P_sort(2))
                        IS_INTERSECT_FLAG=1;
                        plot(X_INTERSECT,Y_INTERSECT,'c+');
                        disp('xxx5');
                        break;
                    end
                end
            end

        end
    end


end






















