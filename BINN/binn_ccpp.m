% function [path, visit_nodes] = binn_ccpp(map, MAX_X, MAX_Y)
%     % 提取起始点
%     xStart = map(1,1); 
%     yStart = map(1,2);
% 
%     % 构建二维栅格地图：0为未覆盖，1为障碍物，2为已覆盖
%     grid_map = zeros(MAX_X, MAX_Y);
%     for i = 2:size(map,1)-1
%         grid_map(map(i,1), map(i,2)) = 1;
%     end
% 
%     % 论文中给出的 BINN 分流方程参数设置
%     A = 10; B = 1; D = 1; E = 100; c_param = 0.5; % c为移动规则相关的权重常数
% 
%     % 神经元活性值初始化
%     x_act = zeros(MAX_X, MAX_Y);
% 
%     curr_x = xStart; 
%     curr_y = yStart;
%     path = [curr_x, curr_y];
%     grid_map(curr_x, curr_y) = 2; % 标记起始点为已覆盖
%     visit_nodes = [];
% 
%     % 8邻域搜索方向及权值计算
%     dx = [-1, 0, 1, -1, 1, -1, 0, 1];
%     dy = [-1, -1, -1, 0, 0, 1, 1, 1];
%     w = zeros(1,8);
%     for k = 1:8
%         w(k) = 1 / norm([dx(k), dy(k)]);
%     end
% 
%     last_dir_x = 0; last_dir_y = 1; % 初始默认移动方向
%     unvisited_left = sum(sum(grid_map == 0));
% 
%     while unvisited_left > 0
%         % 1. 更新环境输入 I_i
%         I = zeros(MAX_X, MAX_Y);
%         I(grid_map == 0) = E;
%         I(grid_map == 1) = -E;
%         I(grid_map == 2) = 0; 
% 
%         I_plus = max(I, 0);
%         I_minus = max(-I, 0);
% 
%         % 2. 神经元活性值局部迭代
%         for iter = 1:5
%             x_new = x_act;
%             for i = 1:MAX_X
%                 for j = 1:MAX_Y
%                     if grid_map(i,j) == 1
%                         x_new(i,j) = -0.9; % 障碍物的活性值为极小值
%                         continue;
%                     end
% 
%                     sum_wx = 0;
%                     for k = 1:8
%                         nx = i + dx(k); ny = j + dy(k);
%                         if nx>=1 && nx<=MAX_X && ny>=1 && ny<=MAX_Y
%                             sum_wx = sum_wx + w(k) * max(x_act(nx,ny), 0);
%                         end
%                     end
% 
%                     % 代入方程计算活性值变化
%                     dx_dt = -A*x_act(i,j) + (B - x_act(i,j))*(I_plus(i,j) + sum_wx) - (D + x_act(i,j))*I_minus(i,j);
%                     x_new(i,j) = x_act(i,j) + 0.1 * dx_dt;
%                 end
%             end
%             x_act = x_new;
%         end
% 
%         % 3. 基于活性值与移动规则的下一节点选择
%         best_val = -inf;
%         next_x = -1; next_y = -1;
% 
%         for k = 1:8
%             nx = curr_x + dx(k); ny = curr_y + dy(k);
%             if nx>=1 && nx<=MAX_X && ny>=1 && ny<=MAX_Y && grid_map(nx,ny) == 0
%                 dir_x = nx - curr_x; dir_y = ny - curr_y;
% 
%                 % 计算方向偏移角 y_j
%                 dot_prod = dir_x*last_dir_x + dir_y*last_dir_y;
%                 norms = norm([dir_x, dir_y]) * norm([last_dir_x, last_dir_y]) + 1e-6;
%                 angle_diff = acos(dot_prod / norms);
%                 y_j = 1 - angle_diff / pi;
% 
%                 % 综合神经元活性值与转向优先规则选点
%                 val = x_act(nx,ny) + c_param * y_j;
%                 if val > best_val
%                     best_val = val;
%                     next_x = nx; next_y = ny;
%                 end
%             end
%         end
% 
%         % 4. 死区脱困处理 (结合A*算法脱困)
%         if next_x == -1
%             % 在全局寻找欧式距离最近的未覆盖点作为 A* 算法的目标点
%             [unvisited_x, unvisited_y] = find(grid_map == 0);
%             if isempty(unvisited_x)
%                 break; 
%             end
% 
%             dist = (unvisited_x - curr_x).^2 + (unvisited_y - curr_y).^2;
%             [~, min_idx] = min(dist);
%             target_x = unvisited_x(min_idx);
%             target_y = unvisited_y(min_idx);
% 
%             % 构造临时 Map 用于调用你的 A* 算法接口
%             temp_map = map;
%             temp_map(1,1) = curr_x; temp_map(1,2) = curr_y; 
%             temp_map(size(map,1),1) = target_x; temp_map(size(map,1),2) = target_y;
% 
%             % 【修改处：修复接口调用参数错误】
%             astar_path = A_star_search(temp_map, MAX_X, MAX_Y);
% 
%             if ~isempty(astar_path)
%                 % 你的A*接口返回的path包含了起点，所以从2开始避免重复记录当前点
%                 for p = 2:size(astar_path, 1)
%                     rx = astar_path(p, 1); ry = astar_path(p, 2);
%                     path = [path; rx, ry];
%                     if grid_map(rx, ry) == 0
%                         grid_map(rx, ry) = 2; % 标记脱困路径上经过的未覆盖点为已覆盖
%                     end
%                     visit_nodes = [visit_nodes; rx, ry];
%                 end
%             else
%                 % A*无解时的降级处理
%                 path = [path; target_x, target_y];
%                 grid_map(target_x, target_y) = 2;
%                 visit_nodes = [visit_nodes; target_x, target_y];
%             end
% 
%             curr_x = target_x; curr_y = target_y;
%             last_dir_x = 0; last_dir_y = 1; % 脱困后重置基准方向
%         else
%             % 正常步进
%             last_dir_x = next_x - curr_x;
%             last_dir_y = next_y - curr_y;
%             curr_x = next_x; curr_y = next_y;
%             path = [path; curr_x, curr_y];
%             grid_map(curr_x, curr_y) = 2;
%             visit_nodes = [visit_nodes; curr_x, curr_y];
%         end
% 
%         unvisited_left = sum(sum(grid_map == 0));
%     end
% end
function [path, visit_nodes, deadlocks, x_act_history] = binn_ccpp(map, MAX_X, MAX_Y)
    % 自动校准边界
    real_max_x = max([MAX_X, max(round(map(:,1)))]);
    real_max_y = max([MAX_Y, max(round(map(:,2)))]);
    MAX_X = real_max_x; MAX_Y = real_max_y;
    
    xStart = round(map(1,1)); yStart = round(map(1,2));
    grid_map = zeros(MAX_X, MAX_Y);
    for i = 2:size(map,1)-1
        grid_map(round(map(i,1)), round(map(i,2))) = 1;
    end
    
    % BINN 参数优化，防止数值积分爆炸
    A = 10; B = 1; D = 1; E = 50; c_param = 0.5; 
    dt = 0.01; % 【关键修复】极小化步长以确保欧拉迭代稳定
    
    x_act = zeros(MAX_X, MAX_Y);
    x_act_history = [];
    step_count = 0;
    
    curr_x = xStart; curr_y = yStart;
    path = [curr_x, curr_y];
    grid_map(curr_x, curr_y) = 2; 
    visit_nodes = [1, curr_x, curr_y]; 
    deadlocks = []; 
    
    dx = [-1, 0, 1, -1, 1, -1, 0, 1]; dy = [-1, -1, -1, 0, 0, 1, 1, 1];
    w = 1./sqrt(dx.^2 + dy.^2);
    last_dir_x = 0; last_dir_y = 1;
    unvisited_left = sum(sum(grid_map == 0));
    
    while unvisited_left > 0
        I = zeros(MAX_X, MAX_Y);
        I(grid_map == 0) = E; I(grid_map == 1) = -E; I(grid_map == 2) = 0; 
        I_plus = max(I, 0); I_minus = max(-I, 0);
        
        % 【关键修复】增加迭代次数保证活性扩散，同时使用边界钳制
        for iter = 1:10 
            x_new = x_act;
            for i = 1:MAX_X
                for j = 1:MAX_Y
                    if grid_map(i,j) == 1, x_new(i,j) = -0.9; continue; end
                    sum_wx = 0;
                    for k = 1:8
                        nx = i + dx(k); ny = j + dy(k);
                        if nx>=1 && nx<=MAX_X && ny>=1 && ny<=MAX_Y
                            sum_wx = sum_wx + w(k) * max(x_act(nx,ny), 0);
                        end
                    end
                    % 动力学方程
                    dx_dt = -A*x_act(i,j) + (B - x_act(i,j))*(I_plus(i,j) + sum_wx) - (D + x_act(i,j))*I_minus(i,j);
                    x_new(i,j) = x_act(i,j) + dt * dx_dt;
                    
                    % 强制截断，避免计算机浮点溢出产生 NaN
                    x_new(i,j) = max(-D, min(B, x_new(i,j))); 
                end
            end
            x_act = x_new;
        end
        
        step_count = step_count + 1;
        x_act_history(:,:,step_count) = x_act;
        
        best_val = -inf; next_x = -1; next_y = -1;
        for k = 1:8
            nx = curr_x + dx(k); ny = curr_y + dy(k);
            if nx>=1 && nx<=MAX_X && ny>=1 && ny<=MAX_Y && grid_map(nx,ny) == 0
                dir_x = nx - curr_x; dir_y = ny - curr_y;
                y_j = 1 - acos((dir_x*last_dir_x + dir_y*last_dir_y)/(norm([dir_x, dir_y])*norm([last_dir_x, last_dir_y])+1e-6))/pi;
                val = x_act(nx,ny) + c_param * y_j;
                if val > best_val, best_val = val; next_x = nx; next_y = ny; end
            end
        end
        
        % 只有真正邻域全是障碍物/已覆盖时才触发脱困机制
        if next_x == -1 
            deadlocks = [deadlocks; curr_x, curr_y];
            [ux, uy] = find(grid_map == 0);
            if isempty(ux), break; end
            [~, midx] = min((ux-curr_x).^2 + (uy-curr_y).^2);
            tx = ux(midx); ty = uy(midx);
            
            t_map = map; t_map(1,1) = curr_x; t_map(1,2) = curr_y; 
            t_map(size(map,1),1) = tx; t_map(size(map,1),2) = ty;
            astar_p = A_star_search(t_map, MAX_X, MAX_Y);
            
            if ~isempty(astar_p)
                for p = 2:size(astar_p, 1)
                    rx = round(astar_p(p,1)); ry = round(astar_p(p,2));
                    path = [path; rx, ry];
                    visit_nodes = [visit_nodes; 1, rx, ry]; 
                    if grid_map(rx, ry) == 0, grid_map(rx, ry) = 2; end
                end
                curr_x = round(astar_p(end,1)); curr_y = round(astar_p(end,2));
            else
                curr_x = tx; curr_y = ty; path = [path; tx, ty];
                visit_nodes = [visit_nodes; 1, tx, ty]; grid_map(tx, ty) = 2;
            end
            last_dir_x = 0; last_dir_y = 1;
        else
            last_dir_x = next_x - curr_x; last_dir_y = next_y - curr_y;
            curr_x = next_x; curr_y = next_y;
            path = [path; curr_x, curr_y];
            visit_nodes = [visit_nodes; 1, curr_x, curr_y];
            grid_map(curr_x, curr_y) = 2;
        end
        unvisited_left = sum(sum(grid_map == 0));
    end
end