function [path, visit_nodes, deadlocks, x_act_history] = binn_ccpp(map, MAX_X, MAX_Y)
    % 自动校准边界
    real_max_x = max([MAX_X, max(round(map(:,1)))]);
    real_max_y = max([MAX_Y, max(round(map(:,2)))]);
    MAX_X = real_max_x; MAX_Y = real_max_y;
    
    % 构建二维栅格地图：0为未覆盖，1为障碍物，2为已覆盖
    xStart = round(map(1,1)); yStart = round(map(1,2));
    grid_map = zeros(MAX_X, MAX_Y);
    for i = 2:size(map,1)-1
        grid_map(round(map(i,1)), round(map(i,2))) = 1;
    end
    
    % BINN 参数优化，防止数值积分爆炸
     %A: 活性值的被动衰减率。

     %B: 神经元活性值的正向饱和上限。

     %D: 神经元活性值的负向饱和下限。

     %E: 未覆盖区域提供的外部刺激强度。

     %c_param: 方向惯性权重系数，用于减少不必要的转弯。

    A = 10; B = 1; D = 1; E = 100; c_param = 0.5; 

    dt = 0.01; % 极小化步长以确保欧拉迭代稳定
    
    %初始化全局神经元的活性值矩阵 x_act（全0）
    % 并初始化用于记录动态变化的三维空矩阵 x_act_history 和步数计数器 step_count
    x_act = zeros(MAX_X, MAX_Y);
    x_act_history = [];
    step_count = 0;
    
    %记录机器人当前坐标
    curr_x = xStart; curr_y = yStart;
    path = [curr_x, curr_y]; %加入路径集合
    grid_map(curr_x, curr_y) = 2;  % 标记起始点为已覆盖
    visit_nodes = [1, curr_x, curr_y]; 
    deadlocks = [];  %初始化死锁点记录数组。
    
    %定义 8 邻域搜索的方向向量
    dx = [-1, 0, 1, -1, 1, -1, 0, 1];
    dy = [-1, -1, -1, 0, 0, 1, 1, 1];

    %计算 8 个方向上的邻接连接权值矩阵 w，权值与欧式距离成反比（对角线距离为根号2，直线距离为1）。
    w = 1./sqrt(dx.^2 + dy.^2);

    %初始化机器人进入地图前的“上一时刻运动方向”，这里预设为沿 Y 轴正方向
    last_dir_x = 0; last_dir_y = 1;
    
    %统计地图中剩余 0（未覆盖区域）的栅格总数，作为主循环是否结束的判断条件。
    unvisited_left = sum(sum(grid_map == 0));%先求列和，再求行和
    

    %主循环
    while unvisited_left > 0
        %每一轮都重新构建一次外部输入矩阵 I：
        % 未覆盖点提供强烈的正向吸引力 E，
        % 障碍物提供强烈的负向排斥力 -E，
        % 已覆盖区域不再提供激励（归 0）。
        I = zeros(MAX_X, MAX_Y);
        I(grid_map == 0) = E; 
        I(grid_map == 1) = -E; 
        I(grid_map == 2) = 0; 

        %将矩阵拆分为正激励矩阵 I_plus 和负激励矩阵 I_minus，以符合 BINN 微分方程的输入规范。
        I_plus = max(I, 0); I_minus = max(-I, 0);
        
        % 增加迭代次数保证活性扩散
        for iter = 1:10
            x_new = x_act;
            for i = 1:MAX_X
                for j = 1:MAX_Y
                    if grid_map(i,j) == 1 %障碍物
                        x_new(i,j) = -0.9; %保持活性值极低为 -0.9 
                        continue; 
                    end
                    sum_wx = 0;
                    for k = 1:8
                        nx = i + dx(k); ny = j + dy(k);
                        if nx>=1 && nx<=MAX_X && ny>=1 && ny<=MAX_Y
                             sum_wx = sum_wx + w(k) * max(x_act(nx,ny), 0); %只累计正的活性值
                        end
                    end
                    % 动力学方程
                    dx_dt = -A*x_act(i,j) + (B - x_act(i,j))*(I_plus(i,j) + sum_wx) - (D + x_act(i,j))*I_minus(i,j);
                    x_new(i,j) = x_act(i,j) + dt * dx_dt;%更新下一刻的活性值
                    
                    % 限幅
                    x_new(i,j) = max(-D, min(B, x_new(i,j))); %活性值最小为 -1，最大为 1
                end
            end
            x_act = x_new;
        end
        
        step_count = step_count + 1;
        x_act_history(:,:,step_count) = x_act;
        
        best_val = -inf; 
        next_x = -1; 
        next_y = -1;
        %移动规则标志位
        flag_up = 0;
        flag_down = 0;
        flag_left = 0;
        flag_right = 0;

        %遍历机器人当前的 8 个邻接点，且仅考察尚未越界并且状态为 0（未覆盖） 的点
        for k = 1:8
            nx = curr_x + dx(k); ny = curr_y + dy(k);
            if nx>=1 && nx<=MAX_X && ny>=1 && ny<=MAX_Y && grid_map(nx,ny) == 0
                dir_x = nx - curr_x; 
                dir_y = ny - curr_y;
                % 计算机器人走向考察点所产生的方向向量 [dir_x, dir_y]。
                % 通过向量点积求该方向与上一时刻运动方向 [last_dir_x, last_dir_y] 夹角的余弦值并转化为角度差。
                % 公式 y_j = 1 - Delta_theta/pi 的作用是：如果保持直走，该值为 1；如果掉头，该值为 0。
                % 这鼓励机器人在空旷区域走直线。

                % Delta_theta = abs(atan((last_dir_x)/(last_dir_y)) - atan((dir_x)/(dir_y)));
                % y_j = 1 - Delta_theta/pi;
                y_j = 1 - acos((dir_x*last_dir_x + dir_y*last_dir_y)/(norm([dir_x, dir_y])*norm([last_dir_x, last_dir_y])+1e-6))/pi;
               
                val = x_act(nx,ny) + c_param * y_j;

                %选出得分最高的相邻未覆盖点，将其坐标记录至 next_x 和 next_y。


                if val > best_val
                    best_val = val; 
                    next_x = nx; 
                    next_y = ny; 
                end
            end
        end


        %% %规则移动法-上
        if(((last_dir_x == 0) && (last_dir_y == 1)) && ...
                ((grid_map(curr_x + dx(8),curr_y + dy(8)) == 1)||(grid_map(curr_x + dx(8),curr_y + dy(8)) == 2)) && ...
                ((grid_map(curr_x + dx(7),curr_y + dy(7)) == 1)||(grid_map(curr_x + dx(7),curr_y + dy(7)) == 2)) && ...
                ((grid_map(curr_x + dx(6),curr_y + dy(6)) == 1)||(grid_map(curr_x + dx(6),curr_y + dy(6)) == 2)) && ...
                ((grid_map(curr_x + dx(4),curr_y + dy(4)) == 1)||(grid_map(curr_x + dx(4),curr_y + dy(4)) == 2)) && ...
                ((grid_map(curr_x + dx(1),curr_y + dy(1)) == 1)||(grid_map(curr_x + dx(1),curr_y + dy(1)) == 2)) && ...
                (grid_map(curr_x + dx(5),curr_y + dy(5)) == 0) && (grid_map(curr_x + dx(3),curr_y + dy(3)) == 0))
            flag_up = 1;
        end

        if(((last_dir_x == 0) && (last_dir_y == 1)) && ...
                ((grid_map(curr_x + dx(8),curr_y + dy(8)) == 1)||(grid_map(curr_x + dx(8),curr_y + dy(8)) == 2)) && ...
                ((grid_map(curr_x + dx(7),curr_y + dy(7)) == 1)||(grid_map(curr_x + dx(7),curr_y + dy(7)) == 2)) && ...
                ((grid_map(curr_x + dx(6),curr_y + dy(6)) == 1)||(grid_map(curr_x + dx(6),curr_y + dy(6)) == 2)) && ...
                ((grid_map(curr_x + dx(5),curr_y + dy(5)) == 1)||(grid_map(curr_x + dx(5),curr_y + dy(5)) == 2)) && ...
                ((grid_map(curr_x + dx(3),curr_y + dy(3)) == 1)||(grid_map(curr_x + dx(3),curr_y + dy(3)) == 2)) && ...
                (grid_map(curr_x + dx(4),curr_y + dy(4)) == 0) && (grid_map(curr_x + dx(1),curr_y + dy(1)) == 0))
            flag_up = 2;
        end

        if(((last_dir_x == 0) && (last_dir_y == 1)) && ...
                ((grid_map(curr_x + dx(1),curr_y + dy(1)) == 1)||(grid_map(curr_x + dx(1),curr_y + dy(1)) == 2)) && ...
                (grid_map(curr_x + dx(4),curr_y + dy(4)) == 0))
            flag_up = 3;
        end
        % 
        % %规则移动法-下
        if(((last_dir_x == 0) && (last_dir_y == -1)) && ...
                ((grid_map(curr_x + dx(1),curr_y + dy(1)) == 1)||(grid_map(curr_x + dx(1),curr_y + dy(1)) == 2)) && ...
                ((grid_map(curr_x + dx(2),curr_y + dy(2)) == 1)||(grid_map(curr_x + dx(2),curr_y + dy(2)) == 2)) && ...
                ((grid_map(curr_x + dx(3),curr_y + dy(3)) == 1)||(grid_map(curr_x + dx(3),curr_y + dy(3)) == 2)) && ...
                ((grid_map(curr_x + dx(4),curr_y + dy(4)) == 1)||(grid_map(curr_x + dx(4),curr_y + dy(4)) == 2)) && ...
                ((grid_map(curr_x + dx(6),curr_y + dy(6)) == 1)||(grid_map(curr_x + dx(6),curr_y + dy(6)) == 2)) && ...
                (grid_map(curr_x + dx(5),curr_y + dy(5)) == 0) && (grid_map(curr_x + dx(8),curr_y + dy(8)) == 0))
            flag_down = 1;
        end

        if(((last_dir_x == 0) && (last_dir_y == -1)) && ...
                ((grid_map(curr_x + dx(1),curr_y + dy(1)) == 1)||(grid_map(curr_x + dx(1),curr_y + dy(1)) == 2)) && ...
                ((grid_map(curr_x + dx(2),curr_y + dy(2)) == 1)||(grid_map(curr_x + dx(2),curr_y + dy(2)) == 2)) && ...
                ((grid_map(curr_x + dx(3),curr_y + dy(3)) == 1)||(grid_map(curr_x + dx(3),curr_y + dy(3)) == 2)) && ...
                ((grid_map(curr_x + dx(5),curr_y + dy(5)) == 1)||(grid_map(curr_x + dx(5),curr_y + dy(5)) == 2)) && ...
                ((grid_map(curr_x + dx(8),curr_y + dy(8)) == 1)||(grid_map(curr_x + dx(8),curr_y + dy(8)) == 2)) && ...
                (grid_map(curr_x + dx(4),curr_y + dy(4)) == 0) && (grid_map(curr_x + dx(6),curr_y + dy(6)) == 0))
            flag_down = 2;
        end  
       
        if(((last_dir_x == 0) && (last_dir_y == -1)) && ...
                ((grid_map(curr_x + dx(6),curr_y + dy(6)) == 1)||(grid_map(curr_x + dx(6),curr_y + dy(6)) == 2)) && ...
                (grid_map(curr_x + dx(4),curr_y + dy(4)) == 0))
            flag_down = 3;
        end    

        %规则移动法-左
        if(((last_dir_x == -1) && (last_dir_y == 0)) && ...
                ((grid_map(curr_x + dx(8),curr_y + dy(8)) == 1)||(grid_map(curr_x + dx(8),curr_y + dy(8)) == 2)) && ...
                (grid_map(curr_x + dx(7),curr_y + dy(7)) == 0))
            flag_left = 1;
        end    
        % 
        % %规则移动法-右

        if(((last_dir_x == 1) && (last_dir_y == 0)) && ...
                ((grid_map(curr_x + dx(6),curr_y + dy(6)) == 1)||(grid_map(curr_x + dx(6),curr_y + dy(6)) == 2)) && ...
                (grid_map(curr_x + dx(7),curr_y + dy(7)) == 0))
            flag_right = 1;
        end  

%%
        
        % 只有邻域全是障碍物/已覆盖时才触发脱困机制
        %如果在上一步的 8 邻域中没有找到任何有效的未覆盖点，说明陷入了死锁，next_x 依然为初值 -1。
        if next_x == -1 
            deadlocks = [deadlocks; curr_x, curr_y];
            %利用 find 查找全图中所有还未覆盖的点坐标 [ux, uy]。如果全图都没有了，直接 break 结束任务。
            [ux, uy] = find(grid_map == 0);
            if isempty(ux)
                break; 
            end
            %计算当前点到所有未覆盖点的欧氏距离平方，寻找距离最近的一个作为脱困的临时终点 [tx, ty]。
            [~, midx] = min((ux-curr_x).^2 + (uy-curr_y).^2);
            tx = ux(midx); ty = uy(midx);
            
            %克隆一个临时的底图 t_map，强制修改其第一行（起点）和最后一行（终点），
            %以符合外部 A_star_search 接口要求的矩阵格式，然后调用 A* 算法求解脱困的穿越路径 astar_p。
            t_map = map; 
            t_map(1,1) = curr_x;
            t_map(1,2) = curr_y; 
            t_map(size(map,1),1) = tx; 
            t_map(size(map,1),2) = ty;
            astar_p = A_star_search(t_map, MAX_X, MAX_Y);
            
            %从索引 2（跳过当前所站起点）开始，将 A* 返回的路径点依次拼接进总体覆盖路径 path 和可视化记录表。
            if ~isempty(astar_p) %成功找到路径
                for p = 2:size(astar_p, 1)
                    rx = round(astar_p(p,1)); 
                    ry = round(astar_p(p,2));
                    path = [path; rx, ry];
                    visit_nodes = [visit_nodes; 1, rx, ry]; 

            % 并在走的过程中，顺便把途径的未覆盖区域一并标记为已覆盖（状态 2）。
                    if grid_map(rx, ry) == 0
                        grid_map(rx, ry) = 2; 
                    end
                end
            % 最后将机器人的当前坐标跳跃到 A* 的终点
                curr_x = round(astar_p(end,1)); 
                curr_y = round(astar_p(end,2));
            else
                fprintf("no path can be found!"); %简单处理：万一 A* 寻路失败（是不可达）
                curr_x = tx; 
                curr_y = ty; 
                path = [path; tx, ty];
                visit_nodes = [visit_nodes; 1, tx, ty]; 
                grid_map(tx, ty) = 2;
            end
            last_dir_x = 0; 
            last_dir_y = 1; %完成一次死区脱困后，将运动惯性方向重置（不再受到死胡同里来回打转的影响）。
        
            %没有陷入死区的正常处理分支
        else
            %%
            % 移动规则上
            if (flag_up == 1)
                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;
                curr_x = next_x; %第一步按照活性值走，人为设定第二步转向
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                % 并将到达的网格状态变更为已覆盖 2。
                grid_map(curr_x, curr_y) = 2;
                % 走两步：第二步设置
                next_x = next_x + dx(2);
                next_y = next_y + dy(2);

                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;

                curr_x = next_x;
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                grid_map(curr_x, curr_y) = 2;
                flag_up = 0;%运行完置零
            elseif(flag_up == 2)
                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;
                curr_x = next_x; 
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                % 并将到达的网格状态变更为已覆盖 2。
                grid_map(curr_x, curr_y) = 2;
                % 走两步
                next_x = next_x + dx(2);
                next_y = next_y + dy(2);

                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;

                curr_x = next_x;
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                grid_map(curr_x, curr_y) = 2;
                flag_up = 0;

            elseif(flag_up == 3)
                next_x = curr_x + dx(4); 
                next_y = curr_y + dy(4);

                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;

                curr_x = next_x;
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                % 并将到达的网格状态变更为已覆盖 2。
                grid_map(curr_x, curr_y) = 2;
                flag_up = 0;


            % 移动规则下
            elseif (flag_down == 1)
                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y; 
                curr_x = next_x; %第一步按照活性值走，人为设定第二步转向
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                % 并将到达的网格状态变更为已覆盖 2。
                grid_map(curr_x, curr_y) = 2;
                % 走两步：第二步设置
                next_x = next_x + dx(7);
                next_y = next_y + dy(7);

                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;

                curr_x = next_x;
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                grid_map(curr_x, curr_y) = 2;
                flag_down = 0;

            elseif(flag_down == 2)
                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;
                curr_x = next_x; 
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                % 并将到达的网格状态变更为已覆盖 2。
                grid_map(curr_x, curr_y) = 2;
                % 走两步
                next_x = next_x + dx(7);
                next_y = next_y + dy(7);

                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;

                curr_x = next_x;
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                grid_map(curr_x, curr_y) = 2;
                flag_down = 0;

            elseif(flag_down == 3)
                next_x = curr_x + dx(4); 
                next_y = curr_y + dy(4);

                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;

                curr_x = next_x;
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                % 并将到达的网格状态变更为已覆盖 2。
                grid_map(curr_x, curr_y) = 2;
                flag_down = 0;

            % 移动规则左
            elseif(flag_left == 1)
                next_x = curr_x + dx(7); 
                next_y = curr_y + dy(7);

                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;

                curr_x = next_x;
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                % 并将到达的网格状态变更为已覆盖 2。
                grid_map(curr_x, curr_y) = 2;
                flag_left = 1;

            % 移动规则右
            elseif(flag_right == 1)
                next_x = curr_x + dx(7); 
                next_y = curr_y + dy(7);

                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;

                curr_x = next_x;
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                % 并将到达的网格状态变更为已覆盖 2。
                grid_map(curr_x, curr_y) = 2;
                flag_right = 1;

            else
                %更新机器人的上一时刻运动方向（用新坐标减去旧坐标）。
                % 然后将当前游标 curr_x, curr_y 推进到选出的最佳点，追加到路径数组中，            
                last_dir_x = next_x - curr_x; 
                last_dir_y = next_y - curr_y;
                curr_x = next_x; 
                curr_y = next_y;
                path = [path; curr_x, curr_y];
                visit_nodes = [visit_nodes; 1, curr_x, curr_y];
                % 并将到达的网格状态变更为已覆盖 2。
                grid_map(curr_x, curr_y) = 2;
            end


            
        end
        %一轮结束，重新盘点全局还有多少个 0，用以决定 while 循环是否在下一轮终止。
        unvisited_left = sum(sum(grid_map == 0));
    end
end