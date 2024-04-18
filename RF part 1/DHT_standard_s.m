function Tran = DHT_standard_s(DH_param)
    theta = DH_param(1);
    alpha = DH_param(2);   
    a = DH_param(3);      
    d = DH_param(4);        
    
    Tran = [cos(theta)     -sin(theta)*cos(alpha)         sin(theta)*sin(alpha)            a*cos(theta);
            sin(theta)     cos(theta)*cos(alpha)          -cos(theta)*sin(alpha)           a*sin(theta);
            0               sin(alpha)                      cos(alpha)                        d;
            0               0                                0                                  1];
end