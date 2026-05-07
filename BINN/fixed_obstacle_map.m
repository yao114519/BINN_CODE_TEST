function map = fixed_obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y)
    map = [xStart, yStart];
    obs = [];
    % 创建一个 U 型死胡同，强制算法触发死区脱困逻辑
    for i = 5:15, obs = [obs; i, 8]; end
    for j = 9:13, obs = [obs; 15, j]; end
    for k = 5:15, obs = [obs; k, 14]; end
    % 过滤掉起终点
    for k = 1:size(obs,1)
        if ~((obs(k,1)==xStart && obs(k,2)==yStart) || (obs(k,1)==xTarget && obs(k,2)==yTarget))
            map = [map; obs(k,:)];
        end
    end
    map = [map; xTarget, yTarget];
end
