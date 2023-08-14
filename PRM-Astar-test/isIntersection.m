%%----------------------------------------------
% Author: Yinsong Qu
% Date:   8,14,2023
% Email: quyinsong@hrbeu.edu.cn
%%----------------------------------------------
function IS_INTERSECT_FLAG = isIntersection( P0,P1,Q )
%ISINTERSECTION 此处显示有关此函数的摘要
%   此处显示详细说明
% P0，P1：起始点和终点
% Q：四边形
% IS_INTERSECT_FLAG：为1则有交叉，为0则没有交叉
IS_INTERSECT_FLAG = 0;
% 判断Q为边数
N = size(Q,2);
% 扩充Q，方便后续相关参数的计算
QQ = [Q,Q(:,1)]; 
% 对P0，P1两个点的x坐标和y坐标分别排序
x_P_sort = sort([P0(1),P1(1)]);
y_P_sort = sort([P0(2),P1(2)]);
% 判断P0和P1是否在Q中
[P0_in,P0_on] = inpolygon(P0(1),P0(2),QQ(1,:),QQ(2,:));
[P1_in,P1_on] = inpolygon(P1(1),P1(2),QQ(1,:),QQ(2,:));
if (P0_in||P0_on) || (P1_in||P1_on)
    IS_INTERSECT_FLAG=1;
    disp('xxx1');
elseif (isequal(P0,P1))
    % 如果两个点都不在Q中，且两个点重合，则不可能和Q相交
    IS_INTERSECT_FLAG=0;
    disp('xxx2');
else
    % 两个点不重合且都不在Q中，计算其与Q交点
    % 计算P0-P1连线参数
    if (P1(1)-P0(1)==0)
        k_P = inf;
        b_P = 0;
    else
        k_P = (P1(2)-P0(2))/(P1(1)-P0(1));
        b_P = P0(2)-k_P*P0(1);
    end
    % 循环四条边判断是否有交叉
    for i=1:N
        % 计算Q的第i条边的参数 y = k_Q*x+b_Q
        if (QQ(1,i+1)-QQ(1,i)==0)
            k_Q(i) = inf;
            b_Q(i) = 0;
        else
            k_Q(i) = (QQ(2,i+1)-QQ(2,i))/(QQ(1,i+1)-QQ(1,i));
            b_Q(i) = QQ(2,i)-k_Q(i)*QQ(1,i);
        end
        % 如果平行则不交叉，但不代表不重合
        if (k_Q(i)==k_P)
            % 如果四点共线，仍然视为相交
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
                % 如果不平行则一定交叉，计算交点
                X_INTERSECT = P0(1);
                Y_INTERSECT = k_Q(i)*X_INTERSECT+b_Q(i);
                % 判断交点是否在两条线段之间
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
                % 如果不平行则一定交叉，计算交点
                X_INTERSECT = Q(1,i);
                Y_INTERSECT = k_P*X_INTERSECT+b_P;
                % 判断交点是否在两条线段之间
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
                % 如果不平行则一定交叉，计算交点
                X_INTERSECT = (b_P-b_Q(i))/(k_Q(i)-k_P);
                Y_INTERSECT = k_Q(i)*X_INTERSECT+b_Q(i);
                % 判断交点是否在两条线段之间
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






















