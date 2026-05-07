function [path,OPEN,CLOSED,k] = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end

    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    OPEN(OPEN_COUNT,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;

    %先扩展初始节点
    exp_array = expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y,xStart,yStart); 
    %将扩展节点加入OPENLIST
    for i=1:size(exp_array,1)
            cx_node=exp_array(i,1);
            cy_node=exp_array(i,2);
            hn=exp_array(i,3);
            gn=exp_array(i,4);
            fn=exp_array(i,5);
          %直接加入openlist
           OPEN_COUNT=OPEN_COUNT+1;
           OPEN(OPEN_COUNT,:)=insert_open(cx_node,cy_node,xStart,yStart,hn,gn,fn);
    end



%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    isInopen =0;
    while(~isempty(OPEN)) %一直遍历
        %找到openlist中fn最小的节点的索引
        i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);

        if i_min == -1
            disp('找不到可行路径！');
            break; 
        end

        %将他作为下一个扩展的节点，并加入closelist
        OPEN(i_min,1)=0;
        xNode=OPEN(i_min,2);        %相当于父节点
        yNode=OPEN(i_min,3);
        CLOSED_COUNT=CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,1)=xNode;
        CLOSED(CLOSED_COUNT,2)=yNode;
        gn=OPEN(i_min,7);

        if(xNode == xTarget && yNode == yTarget)
             break;
        end

        %扩展该最小节点的邻居节点
        exp_array = expand_array(xNode,yNode,gn,xTarget,yTarget,CLOSED,MAX_X,MAX_Y,xStart,yStart);
        
        %将扩展节点加入OPENLIST
        for i=1:size(exp_array,1)
            cx_node=exp_array(i,1);
            cy_node=exp_array(i,2);
            hn=exp_array(i,3);
            gn=exp_array(i,4);
            fn=exp_array(i,5);

            for m=1:size(OPEN,1)    % 判断是否在OPENLIST中,有且gn小就更新，没有就加入
                if(cx_node==OPEN(m,2)&&cy_node==OPEN(m,3))
                      isInopen=1;
                      break;
                else
                      isInopen=0;
                end
            end
%%
           if isInopen      %有且gn小就更新
               n_index = node_index(OPEN,cx_node,cy_node);
               if(gn<OPEN(n_index,7))
                   OPEN(n_index,4)=xNode;
                   OPEN(n_index,5)=yNode;
                   OPEN(n_index,6)=hn;
                   OPEN(n_index,7)=gn;
                   OPEN(n_index,8)=fn;
               end
           else             %否则就加入openlist
               OPEN_COUNT=OPEN_COUNT+1;
               OPEN(OPEN_COUNT,:)=insert_open(cx_node,cy_node,xNode,yNode,hn,gn,fn);
           end

        end

    end %End of While Loop
    %找到已访问节点且被扩展的节点（x,y）坐标和父节点坐标连成线,遍历openlist
    path=[];

    while (1)
        n_index = node_index(OPEN,xNode,yNode);%跳出无限循环最终的节点就是目标点，取出索引
        if(OPEN(n_index,1)==0)%被访问和扩展的节点
            x=OPEN(n_index,2);
            y=OPEN(n_index,3);
            
            if isempty(path)
                path = [x, y];
            else
                path = [x, y; path];
            end
            %找到父节点
            xNode=OPEN(n_index,4);
            yNode=OPEN(n_index,5);

            % 如果到达起点，结束
            if xNode == xStart && yNode == yStart
                %再把初始节点加进去
                path = [xStart, yStart; path];
                break;
            end

        end

    end
end