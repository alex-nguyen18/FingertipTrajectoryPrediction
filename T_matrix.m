%Euler Angles Ry, Rx, Rz
function T=T_matrix(beta,gama,alpha,P)
    beta=deg2rad(beta);gama=deg2rad(gama);alpha=deg2rad(alpha);
    Rx=[1 0 0;
        0 cos(gama) -sin(gama);
        0 sin(gama) cos(gama)];
    
    Ry=[cos(beta) 0 sin(beta);
        0 1 0;
        -sin(beta) 0 cos(beta)];
    
    Rz=[cos(alpha) -sin(alpha) 0;
        sin(alpha) cos(alpha) 0;
        0 0 1];
    
    R=Ry*Rx*Rz;
    
    T=[R(1,1),R(1,2),R(1,3),P(1);
       R(2,1),R(2,2),R(2,3),P(2); 
       R(3,1),R(3,2),R(3,3),P(3);
            0,     0,     0, 1];
end