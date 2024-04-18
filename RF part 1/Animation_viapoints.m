clc
clear all
close all

% 初始化变量
d1 = 100;
l1 = 100;
l2 = 100;
l3 = 100;
psi = -pi/2; % 设定角度

% 定义路径点
viapoints = [150 150 100;
             150 100 150;
             150 50 100;
             150 0 50;
             150 -50 100];

                  
M = 50; % 从一个 viapoint 到下一个 viapoint 的样本数量
n_viapoints = size(viapoints, 1); % 路径点的数量

% 遍历所有 viapoints
for k = 1:n_viapoints
    px = viapoints(k,1);
    py = viapoints(k,2);
    pz = viapoints(k,3);
    q234 = psi;

    % 计算逆运动学
    q1 = atan2(py, px);
    a = d1 - l3*cos(q234) - pz;
    b = px*cos(q1) + py*sin(q1) + l3*sin(q234);
    q3 = real(acos((a^2 + b^2 - l1^2 - l2^2)/(2*l1*l2)));
    q2 = atan2(a*(l1 + l2*cos(q3)) - b*l2*sin(q3), a*l2*sin(q3) + b*(l1 + l2*cos(q3)));
    q4 = q234 - q2 - q3;

    % 存储关节角度
    theta1= q1;
    theta2= q2;
    theta3= q3;
    theta4= q4;
    theta5= 0;

    % 计算转换矩阵
    dh_parameters =[theta1 -pi/2 0 d1;
                    theta2 0 l1 0;
                    theta3 0 l2 0;
                    theta4 -pi/2 0 0;
                    theta5 0 0 l3];

    T01 = DHT_standard_s(dh_parameters(1,:));
    T12 = DHT_standard_s(dh_parameters(2,:));
    T23 = DHT_standard_s(dh_parameters(3,:));
    T34 = DHT_standard_s(dh_parameters(4,:));
    T45 = DHT_standard_s(dh_parameters(5,:));

    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34;
    T05 = T04*T45;

    % 存储每个关节的位置
    position_x(k,:)=[0, T01(1,4),T02(1,4), T03(1,4), T04(1,4), T05(1,4)];
    position_y(k,:)=[0, T01(2,4),T02(2,4), T03(2,4), T04(2,4), T05(2,4)];
    position_z(k,:)=[0, T01(3,4),T02(3,4), T03(3,4), T04(3,4), T05(3,4)];
end

for i = 1:n_viapoints
    % 为每个路径点创建一个新的图形
    figure;
    
    % 在当前图形中绘制机械臂的状态
    plot3(position_x(i,:), position_y(i,:), position_z(i,:), 'o-', 'LineWidth', 2);
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title(sprintf('Robot Arm at Viapoint %d', i)); % 给图形加标题
    text(position_x(i,end), position_y(i,end), position_z(i,end), ['endpoint', num2str(i)]);
    axis equal;
    grid on;
    
    % 保存图形到文件，如果需要的话
    saveas(gcf, sprintf('RobotArm_Viapoint%d.png', i));
    
    % 如果想要在屏幕上看到图形而不是只保存，取消下一行的注释
    % pause(1); % 控制查看图形的时间
end
