function Binn_visualize_results(map, path, visit_nodes, deadlocks, x_act_history, MAX_X, MAX_Y)
    % 获取真实的最大边界
    REAL_MAX_X = max([MAX_X, max(round(map(:,1)))]);
    REAL_MAX_Y = max([MAX_Y, max(round(map(:,2)))]);
    
    % ========================================================
    % 1. 核心指标计算：覆盖率、重复率、转弯次数
    % ========================================================
    % 还原环境栅格图，用于统计自由栅格总数
    grid_map = zeros(REAL_MAX_X, REAL_MAX_Y);
    for i = 2:size(map,1)-1
        grid_map(round(map(i,1)), round(map(i,2))) = 1; % 1为障碍物
    end
    total_free_cells = sum(grid_map(:) == 0);
    
    % 计算覆盖率与重复率
    unique_visited = unique(path, 'rows');
    valid_visited = 0;
    for i = 1:size(unique_visited,1)
        if grid_map(unique_visited(i,1), unique_visited(i,2)) == 0
            valid_visited = valid_visited + 1;
        end
    end
    
    % 覆盖率 = 覆盖到的自由栅格 / 总自由栅格
    coverage_rate = (valid_visited / total_free_cells) * 100;
    % 重复率 = (实际行走总步数 - 覆盖的独特非障碍物数量) / 覆盖的独特数量
    repetition_rate = ((size(path, 1) - valid_visited) / valid_visited) * 100;
    
    % 计算转弯次数
    turns = 0;
    dirs = diff(path);
    % 对每一步的方向向量进行单位化
    for i = 1:size(dirs, 1)
        n = norm(dirs(i,:));
        if n > 0
            dirs(i,:) = dirs(i,:) / n;
        end
    end
    % 若前后两步的单位方向向量发生变化，则记为一次转弯
    for i = 2:size(dirs, 1)
        if norm(dirs(i,:) - dirs(i-1,:)) > 1e-4
            turns = turns + 1;
        end
    end
    
    % 在控制台打印性能指标
    fprintf('====================================\n');
    fprintf('全覆盖路径规划算法性能指标：\n');
    fprintf('覆盖率 (Coverage)   : %.2f%%\n', coverage_rate);
    fprintf('重复率 (Repetition) : %.2f%%\n', repetition_rate);
    fprintf('转弯次数 (Turns)    : %d 次\n', turns);
    fprintf('====================================\n');

    % ========================================================
    % 2. 绘制 2D 路径（重复路径标红）与死锁点
    % ========================================================
    figure('Name', '2D Coverage Path & Deadlocks', 'Color', 'w');
    
    % 先调用你原有的接口绘制底图元素
    visualize_map(map, path, visit_nodes);
    hold on;
    
    % 动态覆盖重绘路径：用以区分是否重复
    visited_counts = zeros(REAL_MAX_X, REAL_MAX_Y);
    visited_counts(path(1,1), path(1,2)) = 1;
    
    for i = 2:size(path, 1)
        p1 = path(i-1, :);
        p2 = path(i, :);
        
        % 判断要进入的下一个点是否已被访问过
        if visited_counts(p2(1), p2(2)) >= 1
            % 如果是重复访问，将该线段重绘为加粗的红色
            plot([p1(1), p2(1)]-0.5, [p1(2), p2(2)]-0.5, 'r', 'LineWidth', 2.5, 'HandleVisibility','off');
        else
            % 首次访问的路径，使用蓝色绘制覆盖
            plot([p1(1), p2(1)]-0.5, [p1(2), p2(2)]-0.5, 'b', 'LineWidth', 1.5, 'HandleVisibility','off');
        end
        % 更新访问计数
        visited_counts(p2(1), p2(2)) = visited_counts(p2(1), p2(2)) + 1;
    end
    
    % 为了图例干净，创建虚拟句柄
    h_opt = plot(NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', '首次覆盖路径');
    h_rep = plot(NaN, NaN, 'r', 'LineWidth', 2.5, 'DisplayName', '重复覆盖路径');
    
    legend_handles = [h_opt, h_rep];
    if ~isempty(deadlocks)
        h_dead = plot(deadlocks(:, 1)-0.5, deadlocks(:, 2)-0.5, 'kp', 'MarkerSize', 14, ...
             'MarkerFaceColor', 'y', 'LineWidth', 1.5, 'DisplayName', '死锁脱困点');
        legend_handles = [legend_handles, h_dead];
    end
    
    % 设置标题显示三大指标
    title(sprintf('Performance: Coverage %.2f%% | Repetition %.2f%% | Turns: %d', ...
          coverage_rate, repetition_rate, turns), 'FontSize', 11, 'FontWeight', 'bold');
    legend(legend_handles, 'Location', 'best');
    hold off;
    
    % ========================================================
    % 3. 动态展示 3D 栅格长方体（神经元激活过程）
    % ========================================================
    if ~isempty(x_act_history)
        fig3d = figure('Name', '3D Neuron Activation Blocks', 'Color', 'w', 'Position', [600, 100, 700, 500]);
        ax3d = axes('Parent', fig3d);
        
        steps = size(x_act_history, 3);
        sample_rate = max(1, floor(steps/60)); 
        
        for s = 1:sample_rate:steps
            if ~ishandle(fig3d), break; end
            cla(ax3d);
            
            h = bar3(ax3d, x_act_history(:,:,s));
            for i = 1:length(h)
                zdata = get(h(i), 'ZData');
                set(h(i), 'CData', zdata, 'FaceColor', 'interp', 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 0.1);
            end
            
            zlim(ax3d, [-1.2, 1.2]); 
            caxis(ax3d, [-1, 1]);
            view(ax3d, -37, 45);
            colormap(ax3d, parula);
            
            title(ax3d, sprintf('Neuron Activity Flow (Step %d/%d)', s, steps));
            xlabel(ax3d, 'Y (Columns)'); 
            ylabel(ax3d, 'X (Rows)'); 
            zlabel(ax3d, 'Neuron Potential');
            
            drawnow;
            pause(0.015); 
        end
        
        if ishandle(fig3d)
            title(ax3d, sprintf('Neuron Activity Flow (Step %d/%d) - Finished', steps, steps));
        end
    end
end