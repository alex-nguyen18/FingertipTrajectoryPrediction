

Rx=[1 0 0;
0 cos(predicted(2)) -sin(predicted(2));
0 sin(predicted(2)) cos(predicted(2))];

Ry=[cos(predicted(3)) 0 sin(predicted(3));
0 1 0;
-sin(predicted(3)) 0 cos(predicted(3))];

Rz=[cos(predicted(1)) -sin(predicted(1)) 0;
sin(predicted(1)) cos(predicted(1)) 0;
0 0 1];

RUpp = Rz*Rx*Ry;
axis = rotm2axang(RUpp);
elbowAng = predicted(4)*180/3.14;
axis(4) = axis(4)*180/3.14;
axisHistory = cat(1,axisHistory,[axis elbowAng]);

Pw{stopFrame};
wristX = (cos(predicted(1))*sin(predicted(2))*cos(predicted(3))-sin(predicted(1))*sin(predicted(3)))*normOFFSET(2)*sin(predicted(4))-(cos(predicted(1))*cos(predicted(2))*normOFFSET(2)*cos(predicted(4)))-cos(predicted(1))*cos(predicted(2))*normOFFSET(1);
wristY = (sin(predicted(1))*sin(predicted(2))*cos(predicted(3))+cos(predicted(1))*sin(predicted(3)))*normOFFSET(2)*sin(predicted(4))-(sin(predicted(1))*cos(predicted(2))*normOFFSET(2)*cos(predicted(4)))-sin(predicted(1))*cos(predicted(2))*normOFFSET(1);
wristZ = (cos(predicted(2))*cos(predicted(3)))*normOFFSET(2)*sin(predicted(4))+sin(predicted(2))*normOFFSET(2)*cos(predicted(4))+sin(predicted(2))*normOFFSET(1);

Pe{stopFrame};
elbowX = -cos(predicted(1))*cos(predicted(2))*normOFFSET(1);
elbowY = -sin(predicted(1))*cos(predicted(2))*normOFFSET(1);
elbowZ = sin(predicted(2))*normOFFSET(1);

predictedElbowTest = cat(1,predictedElbowTest,[elbowX elbowY elbowZ]);
predictedWristTest = cat(1,predictedWristTest,[wristX wristY wristZ]);
%}

%errorData = cat(1,errorData,[abs(predictedESA(n)-joint_angles(5,stopFrame)) abs(Pe{stopFrame}(1)-elbowX) abs(Pe{stopFrame}(2)-elbowY) abs(Pe{stopFrame}(3)-elbowZ)]);