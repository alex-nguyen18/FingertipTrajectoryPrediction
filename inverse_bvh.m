%probably return the jacobian and joint angles
%indices: 3 is wrist, 2 is elbow, and 1 is shoulder
function joint_angles = inverse_bvh(off,tMats,tau)
    %offer rotations and transforms from BVH file
    SToE = tMats{2}(1:3,4) - tMats{1}(1:3,4);
    SToW = tMats{3}(1:3,4) - tMats{1}(1:3,4);
    EToW = tMats{3}(1:3,4) - tMats{2}(1:3,4);
    elbow = pi - LoCosines(SToW,EToW,SToE);

    %Calculate swivel based off of resource
    swivel = swivelAngle(tMats,off,tau);
    
    angles(2) = atan2(SToE(3)/off(1),(1-(SToE(3)/off(1))^2)^(1/2));

    angles(1) = atan2(-SToE(2)/(cos(angles(2))*off(1)),-SToE(1)/(cos(angles(2))*off(1)));

    angles(3) = atan2((SToW(2)*cos(angles(1))-SToW(1)*sin(angles(1)))/(sin(elbow)*off(2)),(SToW(3)-sin(angles(2))*cos(elbow)*off(2)-sin(angles(2))*off(1))/(cos(angles(2))*sin(elbow)*off(2)));
    
    %if earlyMotion == 1
        %angles(3) = -1*swivel;
    %end
    
    joint_angles = [angles(1); angles(2); angles(3); elbow; swivel; -1*swivel];
end

%beta-Y, gamma-X, alpha-Z
function R=eulerMat(gamma,beta,alpha)

    beta=deg2rad(beta);gamma=deg2rad(gamma);alpha=deg2rad(alpha);
    Rx=[1, 0, 0, 0;
        0, cos(gamma), -sin(gamma), 0;
        0, sin(gamma), cos(gamma), 0;
        0,0,0,1];
    
    Ry=[cos(beta), 0, sin(beta), 0;
        0, 1, 0, 0;
        -sin(beta), 0, cos(beta), 0;
        0,0,0,1];
    
    Rz=[cos(alpha), -sin(alpha), 0, 0;
        sin(alpha), cos(alpha), 0, 0;
        0, 0, 1, 0;
        0,0,0,1];
        R=Ry*Rx*Rz;
end

%solves for angle across from a; a, b, and c are triangle's vectors
%input: 3 sides of triangle, a is side opposite theta
%output: theta (radians)
function theta = LoCosines(a,b,c)
    aNorm = norm(a);
    bNorm = norm(b);
    cNorm = norm(c);
    cosTheta = -1*(aNorm^2 - bNorm^2 - cNorm^2)/(2*bNorm*cNorm);
    theta = acos(cosTheta);
end

%solve for elbow swivel angle (angle of arm plane with vertical)
%input: transformation matrices and offsets
%output: swivel (radians)
function swivel= swivelAngle(tMats,off,tau)
    alpha = LoCosines(tMats{3}(1:3,4) - tMats{2}(1:3,4),tMats{3}(1:3,4) - tMats{1}(1:3,4),tMats{2}(1:3,4) - tMats{1}(1:3,4));
    n = (tMats{3}(1:3,4) - tMats{1}(1:3,4))/norm(tMats{3}(1:3,4) - tMats{1}(1:3,4));
    pc = tMats{1}(1:3,4) + off(1) * cos(alpha) * n;
    r = (tMats{2}(1:3,4) - pc) / norm(tMats{2}(1:3,4) - pc);
    %r = (r - dot(r,n)*n)/norm(r - dot(r,n)*n);
    a = [0;-sin(tau*pi/360);-cos(tau*pi/360)]; %tMats{2}(1:3,4) - tMats{3}(1:3,4);
    %}
    u = (a - dot(a,n)*n)/norm(a - dot(a,n)*n);
    v = cross(u,n);
    %R = norm(tMats{2}(1:3,4) - tMats{1}(1:3,4)) * sin(alpha);
    xDrop = dot(v,r);
    yDrop = dot(u,r); %*R;
    swivel = atan2(xDrop,yDrop);
    %{
    if (yDrop >= 0 )
        swivel = acos(yDrop);%/R);
    else
        swivel = pi - acos(yDrop);%/R);
    end
    %}
end