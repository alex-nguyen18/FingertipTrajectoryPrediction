
    X=10*smooth(X',0.1,'moving');Y=10*smooth(Y',0.1,'moving');Z=10*smooth(Z',0.1,'moving'); 
    dx=diff(X);dy=diff(Y);dz=diff(Z);
    phi = rad2deg(phi); %phi is the elbow swivel angle
    
    for j=1:1:size(dx,1)
        dp(j)=norm([dx(j) dy(j) dz(j)]); % norm of the end-effector velocity
    end
    dphi=diff(phi); 
    
    
    for jj=1:1:size(dp,2)
        if abs(dp(jj)*10)<=0.01*max(dp) %set small velocity to be zero
            dp(jj)=0;
        end
        
        if abs(dphi(jj))<=0.01*max(dphi
            dphi(jj)=0;
        end
    end