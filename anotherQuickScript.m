for k=startFrame:stopFrame
    dotp(1,k-startFrame+1) = dot((Pe{k}/norm(Pe{k})),((Pw{k}-Pe{k})/norm(Pw{k}-Pe{k})));
    dotp(2,k-startFrame+1) = dot((Pe{k}/norm(Pe{k})),[0;-1;0]);
    dotp(3,k-startFrame+1) = dot([0;-1;0],(Pw{k}/norm(Pw{k})));
    dotp(4,k-startFrame+1) = joint_angles(4,k)*180/pi;
end
