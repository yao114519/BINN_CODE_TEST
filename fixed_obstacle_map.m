function map = fixed_obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y)
    map = [xStart, yStart];
    obs = [];
    % 创建一个 U 型死胡同，强制算法触发死区脱困逻辑
    % for i = 5:15, obs = [obs; i, 8]; end
    % for j = 9:13, obs = [obs; 15, j]; end
    % for k = 5:15, obs = [obs; k, 14]; end

    %中间障碍物
    for i = 8:11, obs = [obs; i, 8]; end
    for j = 8:11, obs = [obs; j, 9]; end
    for k = 8:11, obs = [obs; k, 10]; end

    for i = 6:7, obs = [obs; i, 19]; end
    for j = 17:19, obs = [obs; 7, j]; end

    for i = 15:16, obs = [obs; i, 2]; end
    for j = 2:5, obs = [obs; 16, j]; end
    % 
    for i = 12:15, obs = [obs; i, 16]; end
    for j = 12:15, obs = [obs; j, 17]; end
    for k = 12:15, obs = [obs; k, 18]; end

    %边界
    for a = 1:20, obs = [obs; a, 1]; end
    for b = 2:19, obs = [obs; 20, b]; end
    for c = 1:20, obs = [obs; c, 20]; end
    for d = 2:19, obs = [obs; 1, d]; end
    % 过滤掉起终点
    for k = 1:size(obs,1)
        if ~((obs(k,1)==xStart && obs(k,2)==yStart) || (obs(k,1)==xTarget && obs(k,2)==yTarget))
            map = [map; obs(k,:)];
        end
    end
    map = [map; xTarget, yTarget];
end