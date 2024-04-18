function new_points = new_trajectory(start_point, end_point, obstacle_center, obstacle_radius, safety_margin,obstacle_height)
    % 计算从起点到终点的向量
    line_vector = end_point - start_point;
    
    % 找到最近的点到障碍物中心的向量
    nearest_vector = obstacle_center - start_point;
    
    % 投影此向量到线路上，找到交点
    t = dot(nearest_vector, line_vector) / norm(line_vector)^2;
    closest_point_on_line = start_point + t * line_vector;
    
    % 检查交点是否在障碍物内部
    if norm(closest_point_on_line(1:2) - obstacle_center(1:2)) <= (obstacle_radius + safety_margin)
        % 计算偏移点
        % 偏移到障碍物旁边的点
        offset_vector = closest_point_on_line(1:2) - obstacle_center(1:2);
        offset_direction = offset_vector / norm(offset_vector);
        side_point = obstacle_center(1:2) + offset_direction * (obstacle_radius + safety_margin);
        side_point = [side_point, start_point(3)]; % 保持原始Z高度
        
        % 偏移到障碍物顶部的点
        top_point = [side_point(1:2), obstacle_center(3) + obstacle_height / 2 + safety_margin];
        
        new_points = [side_point; top_point]; % 返回两个避障点
    else
        new_points = [];
    end
end
