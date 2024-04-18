clc
clear all

px = 0.4950;
py = 2.8075;
pz = 0.3264;
psi = -pi/2;

q234 = psi;
d1 = 1;
l1 = 1;
l2 = 1;
l3 = 1;

q1 = atan2(py, px);
a = d1 - l3*cos(q234) - pz;
b = px*cos(q1) + py*sin(q1) + l3*sin(q234);
q3 = real(acos((a^2 + b^2 - l1^2 - l2^2)/(2*l1*l2)));
q2 = atan2(a*(l1 + l2*cos(q3)) - b*l2*sin(q3), a*l2*sin(q3) + b*(l1 + l2*cos(q3)));
q4 = q234 - q2 - q3;

theta1 = q1;
theta2 = q2;
theta3 = q3;
theta4 = q4;
theta5 = 0;

% Display the theta values
disp('Theta values in radians:');
disp(['theta1: ', num2str(theta1)]);
disp(['theta2: ', num2str(theta2)]);
disp(['theta3: ', num2str(theta3)]);
disp(['theta4: ', num2str(theta4)]);
disp(['theta5: ', num2str(theta5)]);

d2r = pi/180;

dh_parameters =[theta1      pi/2      0       d1;
                theta2      0         l1       0;
                theta3      0         l2       0;
                theta4      -pi/2      0       0;
                theta5      0          0       l3];

T01 = DHT_standard_s(dh_parameters(1,:));
T12 = DHT_standard_s(dh_parameters(2,:));
T23 = DHT_standard_s(dh_parameters(3,:));
T34 = DHT_standard_s(dh_parameters(4,:));
T45 = DHT_standard_s(dh_parameters(5,:));

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;

end_point(:,1) = T05(1:3,4);
position_x(1,:)=[0, T01(1,4),T02(1,4), T03(1,4), T04(1,4), T05(1,4)];
position_y(1,:)=[0, T01(2,4),T02(2,4), T03(2,4), T04(2,4), T05(2,4)];
position_z(1,:)=[0, T01(3,4),T02(3,4), T03(3,4), T04(3,4), T05(3,4)];

% Print the final transformation matrix
disp('The final transformation matrix T05 is:');
disp(T05);

% Plot the robot arm
figure(1)
plot3(position_x, position_y, position_z, 'o-', 'LineWidth', 1);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
text(end_point(1,1), end_point(2,1), end_point(3,1), ['endpoint', num2str(1)]);
axis equal
hold on

grid on
