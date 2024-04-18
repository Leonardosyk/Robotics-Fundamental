close all
clc
clear

theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = 0;
d1 = 1;
l1 = 1;
l2 = 1;
l3 = 1;


% Modelling the robot arm
% Define each linkage and joint type, defaults to rotary joints
%              d        a          alpha         theta
L1 = Link('d', d1, 'a', 0, 'alpha', -pi/2,'offset',0);
L2 = Link('d', 0, 'a', l2, 'alpha', 0,'offset',0);
L3 = Link('d', 0, 'a', l3, 'alpha',0, 'offset', 0);
L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2,'offset', 0);
L5 = Link('d', l3, 'a', 0, 'alpha',0, 'offset',0);

robot=SerialLink([L1,L2,L3,L4,L5]); 
robot.name='lynxmotion robot';
robot.display();
view(3); 

robot.teach

% Calculated workspace
theta1_s=-180;   theta1_end=180;
theta2_s=0;    theta2_end=180;
theta3_s=-90;    theta3_end=90;
theta4_s=-130;    theta4_end=130;
%theta5_s=-180;   theta5_end=180;

% Setting finer step sizes
step = 10; % Smaller step values result in more points being calculated
    step1= (theta1_end -theta1_s) / step;
    step2= (theta2_end -theta2_s) / step;
    step3= (theta3_end -theta3_s) / step;
    step4= (theta4_end -theta4_s) / step;
    %step5= (theta5_end -theta5_s) / step;
    

%Per cent computing workspace
    tic;
    i=1;
    a2r = pi/180;
    num = step1*step2*step3*step4;
    T_x = zeros(1,num);
    T_y = zeros(1,num);
    T_z = zeros(1,num);
        for  q1=theta1_s:step:theta1_end
            for  q2=theta2_s:step:theta2_end
                  for  q3=theta3_s:step:theta3_end
                      for  q4=theta4_s:step:theta4_end
                              %T=robot.fkine([q1*du q2*du q3*du q4*du q5*du 0]);%正向运动学仿真函数
                              T_x(1,i) = cos(q1)*(l2*cos(q2 + q3) + l1*cos(q2) - 1.0*l3*sin(q2 + q3 + q4));
                              T_y(1,i) = -sin(q1)*(d1 - 1.0*l2*cos(q2 + q3) - 1.0*l1*cos(q2) + l3*sin(q2 + q3 + q4));
                              T_z(1,i) = d1 - 1.0*l2*sin(q2 + q3) - 1.0*l1*sin(q2) - 1.0*l3*cos(q2 + q3 + q4);
                              i=i+1;
                      end
                 end
            end 
        end
    

% Mapping the workspace
figure('name','lynxmotion robot workspace')
hold on
plotopt = {'noraise', 'nowrist', 'nojaxes', 'delay',0};
robot.plot([0 0 0 0 0], plotopt{:});
plot3(T_x, T_y, T_z, 'b.', 'MarkerSize', 1);


% Multi-view drawing
for view_angle = 0:45:360
    view(view_angle, 30); % Elevation angle of 30 degrees and azimuth from 0 to 360 degrees
    drawnow; % Update Graphics Window Now
    % Saving the graph of the current viewpoint
    %saveas(gcf, ['workspace_view_' num2str(view_angle) '.png']);
end

% Get X,Y,Z spatial coordinates in the same range.
Point_range = [min(T_x) max(T_x) min(T_y) max(T_y) min(T_z) max(T_z)];
hold off


% Set the five positions and calculate the arm coordinates.

theta1_set= [40 50 60 70 80];
theta2_set= [110 90 70 50 30];
theta3_set= [-110 -90 -70 -50 -30 ];
theta4_set= [-40 -50 -60 -80 -80];
theta5_set= [0 0 0 0 0];

position_x = zeros(6,5);
position_y = zeros(6,5);
position_z = zeros(6,5);
end_point = zeros(3,5);
for i = 1:5
    dh_parameters =[theta1_set(i)       0           0          d1;
                    theta2_set(i)       90         0          0;
                    theta3_set(i)       0           l1         0;
                    theta4_set(i)       0           l2         0;
                    theta5_set(i)       -90         0          0;
                    0                   0           0          l3];

    T01 = DHT_modified_s(dh_parameters(1,:));
    T12 = DHT_modified_s(dh_parameters(2,:));
    T23 = DHT_modified_s(dh_parameters(3,:));
    T34 = DHT_modified_s(dh_parameters(4,:));
    T45 = DHT_modified_s(dh_parameters(5,:));
    T5e = DHT_modified_s(dh_parameters(6,:));

    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34;
    T05 = T04*T45;
    T0e = T05*T5e;
    
    end_point(:,i) = T0e(1:3,4);
    position_x(:,i)=[0, T01(1,4),T02(1,4), T03(1,4), T05(1,4), T0e(1,4)]';
    position_y(:,i)=[0, T01(2,4),T02(2,4), T03(2,4), T05(2,4), T0e(2,4)]';
    position_z(:,i)=[0, T01(3,4),T02(3,4), T03(3,4), T05(3,4), T0e(3,4)]';
end
disp(['Iteration: ', num2str(i)]);
disp(T0e);



% Plot the robot arm
figure ('name','lynxmotion robot model coordinate position') 
for i = 1:5
    plot3(position_x(:,i),position_y(:,i),position_z(:,i),'o-','LineWidth',1);
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    text(end_point(1,i),end_point(2,i),end_point(3,i),['endpoint',num2str(i)]);
    axis equal
    hold on
end
grid on
plot3(end_point(1,1:5),end_point(2,1:5),end_point(3,1:5),'x') 

% Plot the robot arm
figure('Name','Lynxmotion Robot Model Coordinate Position')
% Only plot the last endpoint
i = 5;
plot3(position_x(:,i), position_y(:,i), position_z(:,i), 'o-', 'LineWidth', 1);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
text(end_point(1,i), end_point(2,i), end_point(3,i), ['endpoint', num2str(i)]);
axis equal
grid on


% Solve the equations of motion inversely

px = 200;
py = 100;
pz = 100;
psi = -90;

q234 = psi;


q1 = atan2d(py,px);

nx = cosd(q1)*cosd(q234);
ny = sind(q1)*cosd(q234);
nz = sind(q234);
ax = -cosd(q1)*sind(q234);
ay = -sind(q1)*sind(q234);
az = -cosd(q234);
ox = -sind(q1);
oy = cosd(q1);
oz = 0;
P = [nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];

m = (px+cosd(q1)*sind(q234)*l3)/cosd(q1);
n = pz+cosd(q234)*l3-d1;
e = (m^2+n^2+l1^2-l2^2)/2/l1;
isOutofWorkspace = n^2+m^2-e^2 < 0;
if isOutofWorkspace
    flag = NaN;
    return
end
q2 = [atand(e/sqrt(n^2+m^2-e^2))-atand(m/n),...
    atand(-e/sqrt(n^2+m^2-e^2))-atand(m/n)];

k1 = (m-l1*cosd(q2))/l2;
k2 = (n-l1*sind(q2))/l2;
% theta 3
q3 = atan2d(k2,k1) - q2;
% theta 4
q4 = q234 - q2 - q3;
flag = 1;
[q1 q2(1,1) q3(1,1) q4(1,1);
 q1 q2(1,2) q3(1,2) q4(1,2)];




