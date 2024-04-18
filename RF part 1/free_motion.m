clc;
clear all;
close all;

% Initialise variables
d1 = 100;
l1 = 100;
l2 = 100;
l3 = 100;
psi = -pi/2; % Setting angle

% Define path points
viapoints = [150 150 100;
             150 100 150;
             150 50 100;
             150 0 50;
             150 -50 100];

M = 50; % Number of samples from one viapoint to the next viapoint
n_viapoints = size(viapoints, 1); % Number of waypoints
time_interval = 0.01; % Time interval between points

% Generate time vectors for the entire trajectory
t = linspace(0, time_interval * (M + 1) * (n_viapoints - 1), M * (n_viapoints - 1) + 1);

% Pre-allocated array for storing joint angles
all_theta = [];
all_positions = zeros(3, length(t));


% Generate polynomial coefficients, using polynomials of fourth or lower order
coeffs_x = polyfit(linspace(0, 1, n_viapoints), viapoints(:, 1), 4);
coeffs_y = polyfit(linspace(0, 1, n_viapoints), viapoints(:, 2), 4);
coeffs_z = polyfit(linspace(0, 1, n_viapoints), viapoints(:, 3), 4);

% Initialise the array storing all intermediate points
all_px = [];
all_py = [];
all_pz = [];

% Generate intermediate points on the quintic polynomial trajectory for the whole trajectory
for segment = 1:n_viapoints-1
    for i = 1:M
        t = (segment - 1) + (i-1)/M; % Normalised time variable, range [0, n_viapoints-1]
        normalized_t = t / (n_viapoints - 1); % Normalised to [0, 1] for polynomials
        px = polyval(coeffs_x, normalized_t);
        py = polyval(coeffs_y, normalized_t);
        pz = polyval(coeffs_z, normalized_t);

        % Store all intermediate points
        all_px = [all_px px];
        all_py = [all_py py];
        all_pz = [all_pz pz];
    end
end

% Stores all joint angles and positions
%all_theta = [];
%all_positions = []; 

% Inverse kinematics and loops in animation
for i = 1:M*(n_viapoints-1)
    % Note that t is not redefined here; we index the intermediate points by i
    normalized_t = (i-1) / (M*(n_viapoints-1)); % Normalised time variable, range [0, 1]
    
    % Use of normalised time to evaluate midpoints
    px = polyval(coeffs_x, normalized_t);
    py = polyval(coeffs_y, normalized_t);
    pz = polyval(coeffs_z, normalized_t);
    q234 = psi;

% Inverse kinematics calculations
    q1 = atan2(py, px);
    a = d1 - l3*cos(q234) - pz;
    b = px*cos(q1) + py*sin(q1) + l3*sin(q234);
    q3 = real(acos((a^2 + b^2 - l1^2 - l2^2)/(2*l1*l2)));
    q2 = atan2(a*(l1 + l2*cos(q3)) - b*l2*sin(q3), a*l2*sin(q3) + b*(l1 + l2*cos(q3)));
    q4 = q234 - q2 - q3;
    
    % Storage of joint angles
    theta1 = q1;
    theta2 = q2;
    theta3 = q3;
    theta4 = q4;
    theta5 = 0;
    
    % Calculate conversion matrix
    dh_parameters = [q1 -pi/2 0 d1;
                     q2 0 l1 0;
                     q3 0 l2 0;
                     q4 -pi/2 0 0;
                     psi 0 0 l3];
    
    % Compute the transformation matrices here
    T01 = DHT_standard_s(dh_parameters(1,:));
    T12 = DHT_standard_s(dh_parameters(2,:));
    T23 = DHT_standard_s(dh_parameters(3,:));
    T34 = DHT_standard_s(dh_parameters(4,:));
    T45 = DHT_standard_s(dh_parameters(5,:));
    
    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34;
    T05 = T04*T45;
    
    % Stores the position of each joint for animation
    all_positions(:, i) = [px; py; pz];
    % Add the new theta value to the all_theta array
    all_theta = [all_theta; theta1, theta2, theta3, theta4, theta5];
end




% Robotic arm animation and drawing
figure(1);
hold on;  % Maintains the image so that multiple layers can be added to the same drawing
grid on;  % Open Grid
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('Free Motion');

% Draw path points
plot3(viapoints(:,1), viapoints(:,2), viapoints(:,3), 'go', 'LineWidth', 2, 'MarkerSize', 10);



% labelling of coordinate points, you need to define these points and their labels
for i = 1:n_viapoints
    text(viapoints(i,1), viapoints(i,2), viapoints(i,3), sprintf('(%.0f, %.0f, %.0f)', viapoints(i,:)));
end

axis equal;  % Setting the axes to have equal scale intervals
hold off;  % Release image



