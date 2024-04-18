%% this is to generate the workspace of the parallel robots
R=290*sqrt(3); % side lenth of the triangle base.(mm)
r=130; 
PB1=[0,0];
PB2=[R,0];
PB3=[R/2,sqrt(3)*R/2];

base=[PB1;PB2;PB3;PB1];
plot(base(:,1),base(:,2));
axis([-20,inf,-20,inf])
hold on

%% link lenth (mm)
S=170;
L=130;

a=0.4;
%% traversal of xc,yc
xc = 0:2:506;
yc = 0:2:449;
for i = 1:length(xc)
    for j = 1:length(yc)
        x = xc(i);
        y = yc(j);

        pp1 = [x-r*cos(pi/6+a), y-r*sin(pi/6+a)]; 
        pp2 = [x+r*sin(2*pi/3-a), y+r*cos(2*pi/3-a)];
        pp3 = [x-r*cos(pi/2-a), y+r*sin(pi/2-a)];

        d1 = norm(pp1-PB1);
        d2 = norm(pp2-PB2);
        d3 = norm(pp3-PB3);
        if (d1 < (S+L) && d1 > (S-L) && d2 < (S+L) && d2 > (S-L) && d3 < (S+L) && d3 > (S-L))
            plot(x, y, 'r.')
        end
    end
end

hold off;