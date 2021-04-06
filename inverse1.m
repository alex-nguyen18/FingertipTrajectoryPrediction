function th=inverse1(offset,p_elbow,p_wrist)
N=size(p_elbow,1);

for i=1:N
    th2=atan2((1-(p_elbow(i,3)/(-offset(1,1)))^2)^0.5,p_elbow(i,3)/(-offset(1,1)));
    th1=atan2(p_elbow(i,2)/(offset(1,1)*sin(th2)),p_elbow(i,1)/(offset(1,1)*sin(th2)));
    
    shoulder_wrist=norm(p_wrist(i,:),2);
    th4=atan2(-(1-(((shoulder_wrist)^2-offset(1,1)^2-offset(2,1)^2)/(2*offset(1,1)*offset(2,1)))^2)^0.5,((shoulder_wrist)^2-offset(1,1)^2-offset(2,1)^2)/(2*offset(1,1)*offset(2,1)));

    th3=atan2(-(p_wrist(i,1)*sin(th1) - p_elbow(i,1)*sin(th1) + cos(th1)*(p_elbow(i,2) - p_wrist(i,2)))/(offset(2,1)*sin(th4)),(p_elbow(i,1)*cos(th1) - p_wrist(i,1)*cos(th1) + p_elbow(i,2)*sin(th1) - p_wrist(i,2)*sin(th1) + offset(2,1)*cos(th4)*sin(th2))/(offset(2,1)*cos(th2)*sin(th4)));
    
    
    th(:,i)=[th1; th2; th3; th4];
end 



