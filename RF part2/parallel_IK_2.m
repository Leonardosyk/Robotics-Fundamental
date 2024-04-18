clc
clear all
close all
%% base coordinate
R=290*sqrt(3); % side lenth of the triangle base.(mm)
r=130;%the distance from the vertex to the center of the platform
%coordinate of three vertex of the base
PB1=[0,0];
PB2=[R,0];
PB3=[R/2,sqrt(3)*R/2];
%% link lenth (mm)
%lenth of first arm
SA=170;
SB=170;
SC=170;
%length of second arm
L=130;
%%
%input the cartesian coordinate
xc=300;
yc=180;
a=0.4;
%coordinate of three vertexs of the platform
pp1=[xc-r*cos(pi/6+a),yc-r*sin(pi/6+a)]; 
pp2=[xc+r*sin(2*pi/3-a),yc+r*cos(2*pi/3-a)];
pp3=[xc-r*cos(pi/2-a),yc+r*sin(pi/2-a)];
c1=atan2(yc-r*sin(pi/6+a), xc-r*cos(pi/6+a)); 
c2=atan2(yc+r*cos(2*pi/3-a) -R,xc+r*sin(2*pi/3-a));
c3=atan2(yc+r*sin(pi/2-a)-R*sqrt(3)/2,(xc-r*cos(pi/2-a)-R/2));
d1=acos((SA^2-L^2+norm(pp1-PB1)*norm(pp1-PB1))/(2*SA*norm(pp1-PB1)));
d2=acos((SB^2-L^2+norm(pp2-PB2)^2)/(2*SB*norm(pp2-PB2)));
d3=acos((SC^2-L^2+norm(pp3-PB3)^2)/(2*SC*norm(pp3-PB3)));

theta1=c1+d1;
theta2=c2+d2;
theta3=c3+d3;
%two position of M
M1=[SA*cos(theta1),SA*sin(theta1)];
M2=[R+SB*sin(theta2),SB*cos(theta2)];
M3=[R*0.5+SC*cos(theta3),R*sqrt(3)*0.5+SC*sin(theta3)];
%computed length for the second arm

figure (1) 
title('parallel robot IK')
Base=[PB1;PB2;PB3;PB1];
plot(Base(:,1),Base(:,2),'k-','linewidth',3);
text(PB1(1)-60,PB1(2),'PB1','Color','red','FontSize',14);
text(PB2(1)+10,PB2(2),'PB2','Color','red','FontSize',14);
text(PB3(1),PB3(2)+20,'PB3','Color','red','FontSize',14);
axis([-100 600 -100 500]);
hold on
platform=[pp1;pp2;pp3;pp1];
text(pp1(1),pp1(2),'PP1');
text(pp2(1)+5,pp2(2),'PP2');
text(pp3(1),pp3(2)+5,'PP3');
plot(platform(:,1),platform(:,2),'Color',[0 0.4470 0.7410],'linewidth',2);
fill([pp1(1),pp2(1),pp3(1)],[pp1(2),pp2(2),pp3(2)],[0 0.4470 0.7410]);
link1=[PB1;M1;pp1];
link2=[PB2;M2;pp2];
link3=[PB3;M3;pp3];
text(M1(1),M1(2),'M1');
text(M2(1),M2(2),'M2');
text(M3(1),M3(2),'M3');
plot(link1(:,1),link1(:,2),'r-','linewidth',2);
plot(link2(:,1),link2(:,2),'y-','linewidth',2);
plot(link3(:,1),link3(:,2),'g-','linewidth',2);