% Save the trajectory generation function as StraightTrajectory.m
function trajectory = StraightTrajectory(viapoints, u, M)
    % Assume straight line motion between first two via points
    dist = norm(viapoints(2,:) - viapoints(1,:));
    a = u^2 / (2 * dist); % Constant acceleration
    t1 = u / a; % Time to reach constant speed
    t2 = 2 * dist / u; % Total time for the straight segment

    % Generate time vector
    t_total = linspace(0, t2, M);

    % Preallocate trajectory array
    trajectory = zeros(length(t_total), 3);

    % Generate trajectory
    for i = 1:length(t_total)
        t = t_total(i);
        if t <= t1
            s = 0.5 * a * t^2; % Distance covered during acceleration
        elseif t <= (t2 - t1)
            s = 0.5 * a * t1^2 + u * (t - t1); % Distance covered during constant speed
        else
            s = dist - 0.5 * a * (t2 - t)^2; % Distance covered during deceleration
        end
        % Linear interpolation for the current point
        trajectory(i,:) = viapoints(1,:) + (s / dist) * (viapoints(2,:) - viapoints(1,:));
    end
end