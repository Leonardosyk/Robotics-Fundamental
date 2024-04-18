clc;
clear all;
close all;

% Initialise and define path points
viapoints = [150 150 100;
             150 100 150;
             150 50 100;
             150 0 50;
             150 -50 100];
M = 200; % Number of samples per path segment

% Setting up barriers
obstacle_center = [150, 75, 125]; % Cylindrical centre
obstacle_radius = 20; % Radius
obstacle_height = 50; % Height

% Initialise robot arm parameters
d1 = 100; l1 = 100; l2 = 100; l3 = 100;
psi = -pi/2;

% Generate trajectories and avoid obstacles
adjusted_qset = []; % Store the adjusted trajectory points
safety_margin = 10; % 定义一个安全距离

for i = 1:length(viapoints)-1
    start_point = viapoints(i, :); 
    end_point = viapoints(i+1, :);
    
    % 插入避障点
    avoidance_point = new_trajectory(start_point, end_point, obstacle_center, obstacle_radius, obstacle_height, safety_margin);
    
    if ~isempty(avoidance_point)
        % 如果需要避障，重新定义路径点
        trajectory_points = [start_point; avoidance_point; end_point];
    else
        trajectory_points = [start_point; end_point];
    end
    
    for j = 1:size(trajectory_points, 1) - 1
        segment_start = trajectory_points(j, :);
        segment_end = trajectory_points(j + 1, :);
        for k = 1:M
            t = (k-1)/(M-1);
            current_point = (1-t)*segment_start.' + t*segment_end.';
            adjusted_qset = [adjusted_qset current_point];
        end
    end
end

for i = 1:size(adjusted_qset, 2)
    px = adjusted_qset(1, i);
    py = adjusted_qset(2, i);
    pz = adjusted_qset(3, i);
    q234 = psi;

    % Inverse kinematics calculations
    q1 = atan2(py, px);
    a = d1 - l3*cos(q234) - pz;
    b = px*cos(q1) + py*sin(q1) + l3*sin(q234);
    q3 = real(acos((a^2 + b^2 - l1^2 - l2^2)/(2*l1*l2)));
    q2 = atan2(a*(l1 + l2*cos(q3)) - b*l2*sin(q3), a*l2*sin(q3) + b*(l1 + l2*cos(q3)));
    q4 = q234 - q2 - q3;

    % DH parameters and conversion matrix
    dh_parameters = [q1 -pi/2 0 d1;
                     q2 0 l1 0;
                     q3 0 l2 0;
                     q4 -pi/2 0 0;
                     psi 0 0 l3];

    T01 = DHT_standard_s(dh_parameters(1,:));
    T12 = DHT_standard_s(dh_parameters(2,:));
    T23 = DHT_standard_s(dh_parameters(3,:));
    T34 = DHT_standard_s(dh_parameters(4,:));
    T45 = DHT_standard_s(dh_parameters(5,:));

    % Calculate the continuous transformation matrix
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;

    % Visualisation of current joint position
    position_x = [0, T01(1,4), T02(1,4), T03(1,4), T04(1,4), T05(1,4)];
    position_y = [0, T01(2,4), T02(2,4), T03(2,4), T04(2,4), T05(2,4)];
    position_z = [0, T01(3,4), T02(3,4), T03(3,4), T04(3,4), T05(3,4)];
    plot3(position_x, position_y, position_z, 'o-', 'LineWidth', 1);
end

% Inverse kinematics calculation and visualisation
figure;
hold on;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('5-link Manipulator Trajectory with Obstacle');

% Plotting the adjusted trajectory

plot3(adjusted_qset(1, :), adjusted_qset(2, :), adjusted_qset(3, :), 'r.-');

% Mapping of obstacles
[X, Y, Z] = cylinder(obstacle_radius, 20);
Z = Z * obstacle_height + obstacle_center(3) - obstacle_height / 2;
X = X + obstacle_center(1);
Y = Y + obstacle_center(2);
surf(X, Y, Z, 'FaceColor', 'red', 'FaceAlpha', 0.5);

axis equal; % Ensure consistent scaling of all axes
view(30, 10); % Set a viewing angle to better see the 3D effect

hold off;
