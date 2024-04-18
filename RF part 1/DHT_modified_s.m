function Tran = DHT_modified_s(DH_param)
theta = DH_param(1);
alpha = DH_param(2);   
a = DH_param(3);      
d = DH_param(4);        

Tran = [cosd(theta)        ,-sind(theta),                0,                 a;
    sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -d*sind(alpha);
    sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha), d*cosd(alpha);
    0                       0                        0                      1];
end