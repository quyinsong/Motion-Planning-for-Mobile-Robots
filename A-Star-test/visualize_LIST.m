function visualize_LIST( CLOSED,CLOSED_NUM,OPEN,OPEN_NUM,START,TARGET )
%VISUALIZE_CLOSED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

for i = 1:OPEN_NUM
    Open_Node = OPEN(i,:);
    %�����Start_Node�Ͳ�����OpenNode��
    if Open_Node(1) == START(1) && Open_Node(2) == START(2)
        continue;
    %�����Target_Node�Ͳ�����OpenNode��
    elseif Open_Node(1) == TARGET(1) && Open_Node(2) == TARGET(2)
        continue;
    end
    Open_Node_X = Open_Node(1);
    Open_Node_Y = Open_Node(2);
    fill([Open_Node_X,Open_Node_X,Open_Node_X-1,Open_Node_X-1,Open_Node_X],[Open_Node_Y,Open_Node_Y-1,Open_Node_Y-1,Open_Node_Y,Open_Node_Y],'red'); 
end

for i = 1:CLOSED_NUM
    Close_Node = CLOSED(i,:);
    %�����Start_Node�Ͳ�����CloseNode��
    if Close_Node(1) == START(1) && Close_Node(2) == START(2)
        continue;
    %�����Target_Node�Ͳ�����CloseNode��
    elseif Close_Node(1) == TARGET(1) && Close_Node(2) == TARGET(2)
        continue;
    end
    Close_Node_X = Close_Node(1);
    Close_Node_Y = Close_Node(2);
    fill([Close_Node_X,Close_Node_X,Close_Node_X-1,Close_Node_X-1,Close_Node_X],[Close_Node_Y,Close_Node_Y-1,Close_Node_Y-1,Close_Node_Y,Close_Node_Y],'blue'); 
end

end

