%Return angles based off of wrist position, joint lengths, and swivel angle
function joint_angles = inverse_swivel(swivel,off,wrist,tau)
    %offer rotations and transforms from BVH file
    %unit vector from shoulder to wrist
    %swivel
    nVec = (wrist)/norm(wrist);
    a = [0;-sin(tau*pi/360);-cos(tau*pi/360)];
    u = (a - dot(a,nVec)*nVec)/norm(a - dot(a,nVec)*nVec);
    %angle from Shoulder-Wrist line to Elbow measured at shoulder
    alpha = LoCosines(off(2),wrist,off(1));
    %Perpindicular point on Shoulder-Wrist corresponding to elbow
    c = nVec*(norm(off(1))*cos(alpha));
    bot = c + u*sin(alpha)*off(1);
    %plot(c(1),c(3),'-s');
    rotm = axang2rotm([nVec(1) nVec(2) nVec(3) -swivel]);
    %proj = [wrist(1);wrist(3)]/norm([wrist(1);wrist(3)]);
    %Vector position of elbow relative to C using swivel
    %upper length*sin(alpha) is length of C to shoulder
    %Vector is swivel vector rotated 90 off from n (along Shoulder-Wrist)
    %Combine XZ projection with yDrop from swivel angle
    %swivelVector = norm(off(1))*sin(alpha)*[-sin(swivel)*proj(2);-cos(swivel);-sin(swivel)*proj(1)];
    elbow = rotm * bot;
    %plot(elbow(1),elbow(3),'x');
    %theta4 = -(pi - LoCosines(wrist,off(1),off(2))) + pi/2;
    SToE = elbow;
    SToW = wrist;
    EToW = wrist - elbow;
    
    theta = pi - LoCosines(SToW,EToW,SToE);
    
    angles(2) = atan2(SToE(3)/off(1),(1-(SToE(3)/off(1))^2)^(1/2));

    angles(1) = atan2(-SToE(2)/(cos(angles(2))*off(1)),-SToE(1)/(cos(angles(2))*off(1)));

    angles(3) = atan2((SToW(2)*cos(angles(1))-SToW(1)*sin(angles(1)))/(sin(theta)*off(2)),(SToW(3)-sin(angles(2))*cos(theta)*off(2)-sin(angles(2))*off(1))/(cos(angles(2))*sin(theta)*off(2)));
    %angles(3) = atan2((SToW(2)*cos(angles(1))-SToW(1)*sin(angles(1)))/(cos(elbow)*off(2)),(SToW(3)-sin(angles(2))*cos(elbow)*off(2)+sin(angles(2))*off(1))/(cos(angles(2))*sin(elbow)*off(2)));

    if(dot(nVec,(elbow/norm(elbow)))>.9)
        earlyMotion = 1;
    else
        earlyMotion = 0;
    end
    
    joint_angles = [angles(1) angles(2) angles(3) theta swivel earlyMotion];
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