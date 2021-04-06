clear;close all;clc;
%dataTable = zeros(1,12);
dataTable = zeros(1,11);
%frameData = zeros(1,3);
history = zeros(4,1);
maxVel = zeros(3,1);
%targetWristPositions = zeros(1,3);
%targetElbowPositions = zeros(1,3);
predictedWristTest = zeros(1,3);
predictedElbowTest = zeros(1,3);
xPosLogisticData = zeros(1,5);
yPosLogisticData = zeros(1,5);
zPosLogisticData = zeros(1,5);
%predictedJointsTest = zeros(1,5);
%targetJointsTest = zeros(1,5);
%targetESA = zeros(1,1);
% linkLengths = zeros(1,2);
maxX = 0;
minX = 0;
maxY = 0;
minY = 0;
maxZ = 0;
minZ = 0;
errorData = zeros(1,4);
motionFrames = zeros(1,1);
file=dir('ULTest\*.bvh'); %set xxxx to the directory name; returns all files in dir
%%{
velFunc = fittype('-1*(b*e*(a-d)*(x/c).^b)/(x*((x/c).^b+1)^(e+1))');
accFunc = fittype('-1*(b^2*(-e-1)*e*(a-d)*(x/c).^(2*b-2))/(c^2*((x/c).^b+1)^(e+2))-((b-1)*b*e*(a-d)*(x/c).^(b-2))/(c*c*((x/c).^b+1)^(e+1))');
jerkFunc = fittype('-1*(b^3*(-e-2)*(-e-1)*e*(a-d)*(x/c).^(3*b-3))/(c^3*((x/c).^b+1)^(e+3))-((b-1)*b^2*(-e-1)*e*(a-d)*(x/c).^(2*b-3))/(c^3*((x/c).^b+1)^(e+2))-(b^2*(2*b-2)*(-e-1)*e*(a-d)*(x/c).^(2*b-3))/(c^3*((x/c).^b+1)^(e+2))-(b*(b-2)*(b-1)*e*(a-d)*(x/c).^(b-3))/(c^3*((x/c).^b+1)^(e+1))');
axisHistory = zeros(1,5);
load('predictedESATest.mat');
load('LLPredictedLogisticDataTau.mat');
load('LRPredictedLogisticDataTau.mat');
load('ULPredictedLogisticDataTau.mat');
load('URPredictedLogisticDataTau.mat');
load('ESAModel20Atan.mat');
load('ESAModel40Atan.mat');
load('ESAModel60Atan.mat');
load('ESAModel80Atan.mat');
load('ESAModel100Atan.mat');
load('motionFramePredictionModelFullStartPos');
%%}
%load('targetWristPositionTest.mat');
stopFrames = 0;
for n=3 %1:1:length(file)
    clearvars -except file dataTable ActionName n history velFunc accFunc jerkFunc maxVel stopFrames axisHistory randints targetWristPositions linkLengths targetElbowPositions targetESA predictedWristTest predictedElbowTest predictedESATest predictedJointsTest targetJointsTest errorData xPosLogisticData yPosLogisticData zPosLogisticData minX maxX minY maxY minZ maxZ motionFrames LLPredictedLogisticDataTau LRPredictedLogisticDataTau ULPredictedLogisticDataTau URPredictedLogisticDataTau xPosError yPosError zPosError ESAModel20Atan ESAModel40Atan ESAModel60Atan ESAModel80Atan ESAModel100Atan err_sampled motionFramePredictionModelFullStartPos predictionError%frameData
    filename=['ULTest\',file(n).name]; %set xxxx to directory that holds file
    %filename=['BVH_1700\','LL_50cm_Point9_03_ZH.bvh'];
    %figure(n)
    %set(n,'Units','centimeters','position',[1,2,30,23]);
    fileID=fopen(filename,'r');
    info=textscan(fileID,'%s');  fclose(fileID);
    %OFFSET will store the offset values from the BVH file
    %check the info cell for strings, pick out the ones we need
    OFFSET=[str2num(info{1}{6}),str2num(info{1}{7}),str2num(info{1}{8});
        str2num(info{1}{115}),str2num(info{1}{116}),str2num(info{1}{117}); % spine 相对于 hips 的偏置
        str2num(info{1}{127}),str2num(info{1}{128}),str2num(info{1}{129}); 
        str2num(info{1}{139}),str2num(info{1}{140}),str2num(info{1}{141}); 
        str2num(info{1}{198}),str2num(info{1}{199}),str2num(info{1}{200}); 
        str2num(info{1}{210}),str2num(info{1}{211}),str2num(info{1}{212}); 
        str2num(info{1}{222}),str2num(info{1}{223}),str2num(info{1}{224}); 
        str2num(info{1}{234}),str2num(info{1}{235}),str2num(info{1}{236}); % RightHand 相对于 RightForeArm 的偏置
        -18,0,0 ];  
    frames = str2num(info{1}{878}); %defined after Hierarchy
    timePerFrame = str2num(info{1}{881}); 
    ang=cell(frames,180);
    for e=1:1:frames
        for j=1:1:180
            ang=info{1}(881+181*(e-1)+j);
            A(e,j)=str2num(ang{1});
            if abs(A(e,j))<0.0001
                A(e,j)=0; %channel data saved in A
            end
        end
    end
    i=1; %Delayed incrementer from for loop
    m=1; %step size (every frame as opposed to every other frame, etc.)
    for k=1:m:frames %builds up frame from hip                                       %offset, rotation
        T0_1{i,1}=T_matrix(0,0,0,OFFSET(1,1:3));                                     %hip, base
        T1_2{i,1}=T_matrix(A(k,3+1*3+1),A(k,3+1*3+2),A(k,3+1*3+3),OFFSET(2,1:3));    %spine, hip
        T2_3{i,1}=T_matrix(A(k,3+8*3+1),A(k,3+8*3+2),A(k,3+8*3+3),OFFSET(3,1:3));    %spine1, spine
        T3_4{i,1}=T_matrix(A(k,3+9*3+1),A(k,3+9*3+2),A(k,3+9*3+3),OFFSET(4,1:3));    %spine2, spine1
        T4_5{i,1}=T_matrix(A(k,3+10*3+1),A(k,3+10*3+2),A(k,3+10*3+3),OFFSET(5,1:3)); %shoulder, spine2
        T5_6{i,1}=T_matrix(A(k,3+14*3+1),A(k,3+14*3+2),A(k,3+14*3+3),OFFSET(6,1:3)); %rightarm, shoulder
        T6_7{i,1}=T_matrix(A(k,3+15*3+1),A(k,3+15*3+2),A(k,3+15*3+3),OFFSET(7,1:3)); %forearm, rightarm
        T7_8{i,1}=T_matrix(A(k,3+16*3+1),A(k,3+16*3+2),A(k,3+16*3+3),OFFSET(8,1:3)); %RHIndex3, forearm
        T8_9{i,1}=T_matrix(A(k,3+24*3+1),A(k,3+24*3+2),A(k,3+24*3+3),OFFSET(9,1:3)); %RHMiddle, RHIndex3
        T9_10{i,1}=T_matrix(A(k,3+25*3+1),A(k,3+25*3+2),A(k,3+25*3+3),[0 0 0]);      %[None], RHMiddle
        
        %Current frame multiply to do forward kinematics
        T0_6{i,1}=T0_1{i,1}*T1_2{i,1}*T2_3{i,1}*T3_4{i,1}*T4_5{i,1}*T5_6{i,1};
        T0_7{i,1}=T0_6{i,1}*T6_7{i,1};
        T0_8{i,1}=T0_7{i,1}*T7_8{i,1};
        T0_9{i,1}=T0_8{i,1}*T8_9{i,1};
        T0_10{i,1}=T0_9{i,1}*T9_10{i,1};
        %Don't need T0_9 because last trans just updates the rotation
        %{
        T0_5{i,1}=T0_1{i,1}*T1_2{i,1}*T2_3{i,1}*T3_4{i,1}*T4_5{i,1};
        T0_4{i,1}=T0_1{i,1}*T1_2{i,1}*T2_3{i,1}*T3_4{i,1};
        T0_3{i,1}=T0_1{i,1}*T1_2{i,1}*T2_3{i,1};
        T0_2{i,1}=T0_1{i,1}*T1_2{i,1};
        T0_1{i,1}=T0_1{i,1}; %redundant, but shows the structure
        %}
        PS{i,1}=T0_6{i}(1:3,4); %Now, relative to shoulder
        PE{i,1}=T0_7{i}(1:3,4);
        PW{i,1}=T0_8{i}(1:3,4);
        PM{i,1}=T0_10{i}(1:3,4);
        Ps{i,1}=PS{i,1}-PS{i,1}; %Difference relative to shoulder pos in hip frame
        Pe{i,1}=PE{i,1}-PS{i,1};
        Pw{i,1}=PW{i,1}-PS{i,1};
        Pm{i,1}=PM{i,1}-PS{i,1};
        %Store all data in x,y,z
        x(i,:)=[Ps{i}(1) Pe{i}(1) Pw{i}(1) Pm{i}(1)];
        y(i,:)=[Ps{i}(2) Pe{i}(2) Pw{i}(2) Pm{i}(2)];
        z(i,:)=[Ps{i}(3) Pe{i}(3) Pw{i}(3) Pm{i}(3)];
        X(i)=Pm{i}(1); %end effector quickly accessible in X,Y,Z
        Y(i)=Pm{i}(2);
        Z(i)=Pm{i}(3);
        i=i+1;
    end
    normOFFSET = [norm(OFFSET(7,:));norm(OFFSET(8,:))];
    %th = zeros(4,frames);

    %these measurements are in reference to an absolute value
    %spd/velocity between frames
    for f=1:1:frames-1
        distFrame(f,1) = norm(Pw{f+1,1} - Pw{f,1});
        distFrameX(f,1) = Pw{f+1,1}(1) - Pw{f,1}(1);
        distFrameY(f,1) = Pw{f+1,1}(2) - Pw{f,1}(2);
        distFrameZ(f,1) = Pw{f+1,1}(3) - Pw{f,1}(3);
    end

    startMove = 0;
    totalDist = 0;
    startFrame = 0;
    stopFrame = 0;
    maxFrame = 1;
    ndx = 5;
    numFrames = 0;
    dataTableTemp = zeros(1,12);
    while ndx < frames
        if startMove == 0 && distFrame(ndx) > .075
            startMove = 1;
            startFrame = ndx;
            maxFrame = ndx;
        end
        if startMove == 1 
            %{
            rWristS = norm(Pw{ndx});
            rWristC = (Pw{ndx,1}(1)^2+Pw{ndx,1}(3)^2)^.5;
            thetaWrist = acos(Pw{ndx,1}(1)/rWristC);
            phiWrist = acos(rWristC/rWristS);
            dataTableTemp = cat(1,dataTableTemp,[Pw{ndx}(1) Pw{ndx}(2) Pw{ndx}(3) rWristS phiWrist thetaWrist normOFFSET(1) normOFFSET(2) (normOFFSET(1)+normOFFSET(2)) rWristS/(normOFFSET(1)+normOFFSET(2)) 0 ndx-startFrame]);
            %}
            if distFrame(ndx) > distFrame(maxFrame)
                maxFrame = ndx;
            end
            if ndx - maxFrame > 25 && distFrame(ndx) < .1
                stopFrame = ndx;
                break;
            end
            numFrames = numFrames + 1;
            totalDist = totalDist + distFrame(ndx);
        end
        ndx = ndx + 1;
    end
    
        joint_angles = zeros(6,frames);
    for f=1:1:frames
        %th(:,f) = inverse1(normOFFSET,Pe{f,:}',Pw{f,:}');
        tMats = [T0_6(f,1); T0_7(f,1); T0_8(f,1); T0_10(f,1)];
        if f<startFrame
            tau = 0;
        elseif f>stopFrame
            tau = 1;
        else
            tau = (f-startFrame)/(stopFrame-startFrame+1);
        end
        joint_angles(:,f) = inverse_bvh(normOFFSET,tMats,tau);
        if(f>1)
            if(joint_angles(3,f) > 1+joint_angles(3,f-1))
                joint_angles(3,f) = joint_angles(3,f)-pi;
            elseif(joint_angles(3,f) < joint_angles(3,f-1)-1)
                joint_angles(3,f) = joint_angles(3,f)+pi;
            end
        end
    end
    %dataTableTemp(:,11) = [0 joint_angles(5,startFrame:stopFrame)];
    %{
    dataTableTemp = dataTableTemp(2:size(dataTableTemp,1),:);
    dataTableTemp(:,12) = dataTableTemp(:,12)/(numFrames);
    dataTable = cat(1,dataTable,dataTableTemp);
    %}
    %motionFrames = cat(2,motionFrames,stopFrame - startFrame);
    %dataTable = cat(1,dataTable,[Pw{startFrame}(1) Pw{startFrame}(2) Pw{startFrame}(3) Pw{stopFrame}(1) Pw{stopFrame}(2) Pw{stopFrame}(3) normOFFSET(1) normOFFSET(2) stopFrame-startFrame]);
    rWristS = norm(Pw{stopFrame});
    rWristC = (Pw{stopFrame,1}(1)^2+Pw{stopFrame,1}(3)^2)^.5;
    thetaWrist = acos(Pw{stopFrame,1}(1)/rWristC);
    phiWrist = acos(rWristC/rWristS);
    %rElbowS = norm(Pe{stopFrame});
    %rElbowC = (Pe{stopFrame,1}(1)^2+Pe{stopFrame,1}(3)^2)^.5;
    %thetaElbow = acos(Pe{stopFrame,1}(1)/rElbowC);
    %phiElbow = acos(rElbowC/rElbowS);
    if (Pw{stopFrame}(2) < 0)
        phiWrist = -phiWrist;
    end
    %if (Pe{stopFrame}(2) < 0)
    %    phiElbow = -phiElbow;
    %end
    %px py pz avgWristSpd -> Swivel
    avgspd = totalDist/(numFrames*timePerFrame);
    if (avgspd<10 || abs(sum(normOFFSET)- rWristS)<3)
       continue 
    end
    if (Pw{stopFrame}(3) < 15)
       continue
    end
    
    dataTable = cat(1,dataTable,[Pw{stopFrame}(1) Pw{stopFrame}(2) Pw{stopFrame}(3) rWristS phiWrist thetaWrist avgspd normOFFSET(1)+normOFFSET(2) normOFFSET(1) normOFFSET(2) rWristS/(normOFFSET(1)+normOFFSET(2))]);

    %{
    stopFrames = cat(1,stopFrames,stopFrame);
    
    maxX = cat(1,maxX,max(x(startFrame,3),x(stopFrame,3)));
    minX = cat(1,minX,min(x(startFrame,3),x(stopFrame,3)));
    maxY = cat(1,maxY,max(y(startFrame,3),y(stopFrame,3)));
    minY = cat(1,minY,min(y(startFrame,3),y(stopFrame,3)));
    maxZ = cat(1,maxZ,max(z(startFrame,3),z(stopFrame,3)));
    minZ = cat(1,minZ,min(z(startFrame,3),z(stopFrame,3)));
    %}
    %targetESA = cat(1,targetESA,joint_angles(5,stopFrame));
    %targetElbowPositions = cat(1,targetElbowPositions,transpose(Pe{stopFrame}));
    %targetWristPositions = cat(1,targetWristPositions,transpose(Pw{stopFrame}));
    %linkLengths = cat(1,linkLengths,[normOFFSET(1) normOFFSET(2)]);
    %joint_angles_angles = joint_angles*180/3.14;
    %predicted = inverse_swivel(predictedESA(n),normOFFSET,Pw{stopFrame});
    %targetJointsTest = cat(1,targetJointsTest,transpose(joint_angles_angles(:,stopFrame)));
    %predictedJointsTest = cat(1,predictedJointsTest,predicted*180/3.14);
    
    %predicted = inverse_swivel(predictedESATest(n),normOFFSET,Pw{stopFrame});
    %testElbow = Pw{stopFrame}; 
    %stopFrame
    %disp('actual');
    %Pe{stopFrame}
    %plot(Pe{stopFrame}(1),Pe{stopFrame}(3),'o');
    %plot(Pw{stopFrame}(1),Pw{stopFrame}(3),'o');
    %history = cat(2,history,actual-predicted);
    
    
    %%{
    xSpace = transpose(1/(stopFrame-startFrame+1):1/(stopFrame-startFrame+1):1);
    %xSpace = transpose(1:1:stopFrame-startFrame+1);
    
    
    % The 5PL equation is:
    % F(x) = D+(A-D)/((1+(x/C)^B)^E)
    [cf_x_pos,G_x_pos] = L5P(xSpace,x(startFrame:stopFrame,3));
    [cf_y_pos,G_y_pos] = L5P(xSpace,y(startFrame:stopFrame,3));
    [cf_z_pos,G_z_pos] = L5P(xSpace,z(startFrame:stopFrame,3));
    
    xPosLogisticData = cat(1,xPosLogisticData,[cf_x_pos.A cf_x_pos.B cf_x_pos.C cf_x_pos.D cf_x_pos.E]); 
    yPosLogisticData = cat(1,yPosLogisticData,[cf_y_pos.A cf_y_pos.B cf_y_pos.C cf_y_pos.D cf_y_pos.E]);
    zPosLogisticData = cat(1,zPosLogisticData,[cf_z_pos.A cf_z_pos.B cf_z_pos.C cf_z_pos.D cf_z_pos.E]);
    
    
    %}
    %{
    velX = cfit(velFunc,cf_x_pos.A,cf_x_pos.B,cf_x_pos.C,cf_x_pos.D,cf_x_pos.E);
    velY = cfit(velFunc,cf_y_pos.A,cf_y_pos.B,cf_y_pos.C,cf_y_pos.D,cf_y_pos.E);
    velZ = cfit(velFunc,cf_z_pos.A,cf_z_pos.B,cf_z_pos.C,cf_z_pos.D,cf_z_pos.E);
    
    accX = cfit(accFunc,cf_x_pos.A,cf_x_pos.B,cf_x_pos.C,cf_x_pos.D,cf_x_pos.E);
    accY = cfit(accFunc,cf_y_pos.A,cf_y_pos.B,cf_y_pos.C,cf_y_pos.D,cf_y_pos.E);
    accZ = cfit(accFunc,cf_z_pos.A,cf_z_pos.B,cf_z_pos.C,cf_z_pos.D,cf_z_pos.E);

    jerkX = cfit(jerkFunc,cf_x_pos.A,cf_x_pos.B,cf_x_pos.C,cf_x_pos.D,cf_x_pos.E);
    jerkY = cfit(jerkFunc,cf_y_pos.A,cf_y_pos.B,cf_y_pos.C,cf_y_pos.D,cf_y_pos.E);
    jerkZ = cfit(jerkFunc,cf_z_pos.A,cf_z_pos.B,cf_z_pos.C,cf_z_pos.D,cf_z_pos.E);
    
    %}
    %{
    figure(n)
    title('Workspace Position');
    hold on
    ySpace = cf_x_pos(xSpace);
    plot(xSpace,ySpace,'r');
    plot(xSpace,x(startFrame:stopFrame,3),'y');
    ySpace = cf_y_pos(xSpace);
    plot(xSpace,ySpace,'g');
    plot(xSpace,y(startFrame:stopFrame,3),'m');
    ySpace = cf_z_pos(xSpace);
    plot(xSpace,ySpace,'b');
    plot(xSpace,z(startFrame:stopFrame,3),'c');
    %}
    %{
    figure(2)
    maxVels = [0;0;0];
    title('Workspace Velocity');
    hold on
    ySpace = velX(xSpace);
    maxVels(1,1) = max(ySpace);
    plot(xSpace,ySpace,'r');
    ySpace = velY(xSpace);
    maxVels(2,1) = max(ySpace);
    plot(xSpace,ySpace,'g');
    ySpace = velZ(xSpace);
    maxVels(3,1) = max(ySpace);
    plot(xSpace,ySpace,'b');
    
    maxVel = cat(2,maxVel,maxVels);
    
    
    figure(3)
    title('Workspace Acceleration');
    hold on
    ySpace = accX(xSpace);
    plot(xSpace,ySpace,'r');
    ySpace = accY(xSpace);
    plot(xSpace,ySpace,'g');
    ySpace = accZ(xSpace);
    plot(xSpace,ySpace,'b');
    
    figure(4)
    title('Workspace Jerk');
    hold on
    ySpace = jerkX(xSpace);
    plot(xSpace,ySpace,'r');
    ySpace = jerkY(xSpace);
    plot(xSpace,ySpace,'g');
    ySpace = jerkZ(xSpace);
    plot(xSpace,ySpace,'b');
    %}
    %{
    joint_angles_angles = joint_angles*180/3.14;
    
    % Joint Space
    % The 5PL equation is:
    % F(x) = D+(A-D)/((1+(x/C)^B)^E)
    [cf_j1_pos,G_j1_pos] = L5P(xSpace,transpose(joint_angles_angles(1,startFrame:stopFrame)));
    [cf_j2_pos,G_j2_pos] = L5P(xSpace,transpose(joint_angles_angles(2,startFrame:stopFrame)));
    [cf_j3_pos,G_j3_pos] = L5P(xSpace,transpose(joint_angles_angles(3,startFrame:stopFrame)));
    [cf_j4_pos,G_j4_pos] = L5P(xSpace,transpose(joint_angles_angles(4,startFrame:stopFrame)));
    [cf_j5_pos,G_j5_pos] = L5P(xSpace,transpose(joint_angles_angles(5,startFrame:stopFrame)));
    
    figure(5)
    title('Jointspace Position');
    hold on
    ySpace = cf_j1_pos(xSpace);
    plot(xSpace,ySpace,'r');
    plot(xSpace,joint_angles_angles(1,startFrame:stopFrame),'r');
    ySpace = cf_j2_pos(xSpace);
    plot(xSpace,ySpace,'g');
    plot(xSpace,joint_angles_angles(2,startFrame:stopFrame),'g');
    ySpace = cf_j3_pos(xSpace);
    plot(xSpace,ySpace,'b');
    plot(xSpace,joint_angles_angles(3,startFrame:stopFrame),'b');
    ySpace = cf_j4_pos(xSpace);
    plot(xSpace,ySpace,'y');
    plot(xSpace,joint_angles_angles(4,startFrame:stopFrame),'y');
    ySpace = cf_j5_pos(xSpace);
    plot(xSpace,ySpace,'m');
    plot(xSpace,joint_angles_angles(5,startFrame:stopFrame),'m');
    
    
    velj1 = cfit(velFunc,cf_j1_pos.A,cf_j1_pos.B,cf_j1_pos.C,cf_j1_pos.D,cf_j1_pos.E);
    velj2 = cfit(velFunc,cf_j2_pos.A,cf_j2_pos.B,cf_j2_pos.C,cf_j2_pos.D,cf_j2_pos.E);
    velj3 = cfit(velFunc,cf_j3_pos.A,cf_j3_pos.B,cf_j3_pos.C,cf_j3_pos.D,cf_j3_pos.E);
    velj4 = cfit(velFunc,cf_j4_pos.A,cf_j4_pos.B,cf_j4_pos.C,cf_j4_pos.D,cf_j4_pos.E);
    velj5 = cfit(velFunc,cf_j5_pos.A,cf_j5_pos.B,cf_j5_pos.C,cf_j5_pos.D,cf_j5_pos.E);
    
    figure(6)
    title('Jointspace Velocity');
    hold on
    ySpace = velj1(xSpace);
    plot(xSpace,ySpace,'r');
    ySpace = velj2(xSpace);
    plot(xSpace,ySpace,'g');
    ySpace = velj3(xSpace);
    plot(xSpace,ySpace,'b');
    ySpace = velj4(xSpace);
    plot(xSpace,ySpace,'y');
    ySpace = velj5(xSpace);
    plot(xSpace,ySpace,'m');
    
    accj1 = cfit(accFunc,cf_j1_pos.A,cf_j1_pos.B,cf_j1_pos.C,cf_j1_pos.D,cf_j1_pos.E);
    accj2 = cfit(accFunc,cf_j2_pos.A,cf_j2_pos.B,cf_j2_pos.C,cf_j2_pos.D,cf_j2_pos.E);
    accj3 = cfit(accFunc,cf_j3_pos.A,cf_j3_pos.B,cf_j3_pos.C,cf_j3_pos.D,cf_j3_pos.E);
    accj4 = cfit(accFunc,cf_j4_pos.A,cf_j4_pos.B,cf_j4_pos.C,cf_j4_pos.D,cf_j4_pos.E);
    accj5 = cfit(accFunc,cf_j5_pos.A,cf_j5_pos.B,cf_j5_pos.C,cf_j5_pos.D,cf_j5_pos.E);

    figure(7)
    title('Jointspace Acceleration');
    hold on
    ySpace = accj1(xSpace);
    plot(xSpace,ySpace,'r');
    ySpace = accj2(xSpace);
    plot(xSpace,ySpace,'g');
    ySpace = accj3(xSpace);
    plot(xSpace,ySpace,'b');
    ySpace = accj4(xSpace);
    plot(xSpace,ySpace,'y');
    ySpace = accj5(xSpace);
    plot(xSpace,ySpace,'m');
    
    jerkj1 = cfit(jerkFunc,cf_j1_pos.A,cf_j1_pos.B,cf_j1_pos.C,cf_j1_pos.D,cf_j1_pos.E);
    jerkj2 = cfit(jerkFunc,cf_j2_pos.A,cf_j2_pos.B,cf_j2_pos.C,cf_j2_pos.D,cf_j2_pos.E);
    jerkj3 = cfit(jerkFunc,cf_j3_pos.A,cf_j3_pos.B,cf_j3_pos.C,cf_j3_pos.D,cf_j3_pos.E);
    jerkj4 = cfit(jerkFunc,cf_j4_pos.A,cf_j4_pos.B,cf_j4_pos.C,cf_j4_pos.D,cf_j4_pos.E);
    jerkj5 = cfit(jerkFunc,cf_j5_pos.A,cf_j5_pos.B,cf_j5_pos.C,cf_j5_pos.D,cf_j5_pos.E);
 
    figure(8)
    title('Jointspace Jerk');
    hold on
    ySpace = jerkj1(xSpace);
    plot(xSpace,ySpace,'r');
    ySpace = jerkj2(xSpace);
    plot(xSpace,ySpace,'g');
    ySpace = jerkj3(xSpace);
    plot(xSpace,ySpace,'b');
    ySpace = jerkj4(xSpace);
    plot(xSpace,ySpace,'y');
    ySpace = jerkj5(xSpace);
    plot(xSpace,ySpace,'m');
    if n==10
        display('LL');
    elseif n==20
        display('LR');
    elseif n==30
        display('M');
    elseif n==40
        display('UL');
    elseif n==50
        display('UR');
    end
    %}
    
    %{
    figure(2)
    hold on
    plot(1:size(joint_angles,2),joint_angles_angles(1,:),'r')
    plot(1:size(joint_angles,2),joint_angles_angles(2,:),'g')
    plot(1:size(joint_angles,2),joint_angles_angles(3,:),'b')
    plot(1:size(joint_angles,2),joint_angles_angles(4,:),'m')
    plot(1:size(joint_angles,2),joint_angles_angles(5,:),'y')
    %}
    
    %dataTable = cat(1,dataTable,[Pw{stopFrame}(1) Pw{stopFrame}(2) Pw{stopFrame}(3) rWristS phiWrist thetaWrist normOFFSET(1) normOFFSET(2) (normOFFSET(1)+normOFFSET(2)) rWristS/(normOFFSET(1)+normOFFSET(2)) joint_angles(5,stopFrame)]);
    
    %{
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
    
    %errorData = cat(1,errorData,[abs(predictedESATest(n)-joint_angles(5,stopFrame)) abs(Pe{stopFrame}(1)-elbowX) abs(Pe{stopFrame}(2)-elbowY) abs(Pe{stopFrame}(3)-elbowZ)]);
    %{
    %Min Jerk Model
    motionFrames = stopFrame - startFrame;
    minJerk = zeros(1,3);
    for k=0:1:motionFrames
        tau = k/motionFrames;
        minJerkPos(k+1,:) = transpose(Pw{startFrame})+(transpose(Pw{stopFrame})-transpose(Pw{startFrame}))*(10*tau^3-15*tau^4+6*tau^5);        
        minJerkVel(k+1,:) = (1/motionFrames)*(transpose(Pw{stopFrame})-transpose(Pw{startFrame}))*(30*tau^2-60*tau^3+30*tau^4);
        minJerkAcc(k+1,:) = (1/motionFrames^2)*(transpose(Pw{stopFrame})-transpose(Pw{startFrame}))*(60*tau-180*tau^2+120*tau^3);
        minJerkJerk(k+1,:) = (1/motionFrames^3)*(transpose(Pw{stopFrame})-transpose(Pw{startFrame}))*(60-360*tau+360*tau^2);
    end
    
    figure(100)
    hold on
    plot(1:motionFrames+1,minJerkPos(:,1),'r')
    plot(1:motionFrames+1,minJerkPos(:,2),'g')
    plot(1:motionFrames+1,minJerkPos(:,3),'b')
    plot(1:motionFrames+1,x(startFrame:stopFrame,3),'y');
    plot(1:motionFrames+1,y(startFrame:stopFrame,3),'m');
    plot(1:motionFrames+1,z(startFrame:stopFrame,3),'c');
    
    figure(101)
    hold on
    plot(1:motionFrames+1,minJerkVel(:,1),'r')
    plot(1:motionFrames+1,minJerkVel(:,2),'g')
    plot(1:motionFrames+1,minJerkVel(:,3),'b')
    ySpace = velX(xSpace);
    plot(xSpace,ySpace,'y');
    ySpace = velY(xSpace);
    plot(xSpace,ySpace,'m');
    ySpace = velZ(xSpace);
    plot(xSpace,ySpace,'c');

    figure(102)
    hold on
    plot(1:motionFrames+1,minJerkAcc(:,1),'r')
    plot(1:motionFrames+1,minJerkAcc(:,2),'g')
    plot(1:motionFrames+1,minJerkAcc(:,3),'b')
    ySpace = accX(xSpace);
    plot(xSpace,ySpace,'y');
    ySpace = accY(xSpace);
    plot(xSpace,ySpace,'m');
    ySpace = accZ(xSpace);
    plot(xSpace,ySpace,'c');
    
    figure(103)
    hold on
    plot(1:motionFrames+1,minJerkJerk(:,1),'r')
    plot(1:motionFrames+1,minJerkJerk(:,2),'g')
    plot(1:motionFrames+1,minJerkJerk(:,3),'b')
    ySpace = jerkX(xSpace);
    plot(xSpace,ySpace,'y');
    ySpace = jerkY(xSpace);
    plot(xSpace,ySpace,'m');
    ySpace = jerkZ(xSpace);
    plot(xSpace,ySpace,'c');

    %}
    %{
    if n==10
        display('LL');
    elseif n==20
        display('LR');
    elseif n==30
        display('M');
    elseif n==40
        display('UL');
    elseif n==50
        display('UR');
    end
    %}
    %motionFrames(n) = stopFrame-startFrame;
    %{
    figure(1)
    plot(xSpace,x(startFrame:stopFrame,3),'m');
    hold on;
    ySpace = cf_x_pos(xSpace);
    plot(xSpace,ySpace,'y');
    %motionFrames(n,1) = cf_x_pos.C/(stopFrame-startFrame);
    cf_x_pos.A = minX(n+1);
    cf_x_pos.B = URPredictedLogisticDataTau(n,1);
    cf_x_pos.C = URPredictedLogisticDataTau(n,2);
    cf_x_pos.D = maxX(n+1);
    ySpace = cf_x_pos(xSpace);
    plot(xSpace,ySpace,'c');
    xPosError(n) = max(abs(ySpace - x(startFrame:stopFrame,3)));
    
    
    plot(xSpace,y(startFrame:stopFrame,3),'r');
    ySpace = cf_y_pos(xSpace);
    plot(xSpace,ySpace,'g');
    %motionFrames(n,2) = cf_y_pos.C/(stopFrame-startFrame);
    cf_y_pos.A = minY(n+1);
    cf_y_pos.B = URPredictedLogisticDataTau(n,3);
    cf_y_pos.C = URPredictedLogisticDataTau(n,4);
    cf_y_pos.D = maxY(n+1);
    ySpace = cf_y_pos(xSpace);
    plot(xSpace,ySpace,'b');
    yPosError(n) = max(abs(ySpace - y(startFrame:stopFrame,3)));
    
    plot(xSpace,z(startFrame:stopFrame,3),'r');
    ySpace = cf_z_pos(xSpace);
    plot(xSpace,ySpace,'g');
    %motionFrames(n,3) = cf_z_pos.C/(stopFrame-startFrame);
    cf_z_pos.A = minZ(n+1);
    cf_z_pos.B = URPredictedLogisticDataTau(n,5);
    cf_z_pos.C = URPredictedLogisticDataTau(n,6);
    cf_z_pos.D = maxZ(n+1);
    ySpace = cf_z_pos(xSpace);
    plot(xSpace,ySpace,'b');
    zPosError(n) = max(abs(ySpace - z(startFrame:stopFrame,3)));
    hold off;
    %}
    %{
    quickScript; 
   
    %%{
    endEarlyMotion = int16((endEarlyMotion/100)/(1/(stopFrame-startFrame+1)));
    
    
    ySpace = sampled_cf_j1(xSpace);
    err_sampled(n,1) = max(abs(transpose(joint_angles(1,startFrame:stopFrame)) - ySpace));
    ySpace = sampled_cf_j2(xSpace);
    err_sampled(n,2) = max(abs(transpose(joint_angles(2,startFrame:stopFrame)) - ySpace));
    ySpace = sampled_cf_j3(xSpace);
    err_sampled(n,3) = max(abs(transpose(joint_angles(3,startFrame+endEarlyMotion:stopFrame)) - ySpace(1+endEarlyMotion:stopFrame-startFrame+1)));
    ySpace = sampled_cf_j4(xSpace);
    err_sampled(n,4) = max(abs(transpose(joint_angles(4,startFrame:stopFrame)) - ySpace));
    ySpace = sampled_cf_j5(xSpace);
    err_sampled(n,5) = max(abs(transpose(joint_angles(5,startFrame:stopFrame)) - ySpace));
    %}
    
    %%{
    %predictedFrames = motionFramePredictionModelFull.predictFcn(
    %}
    %{
    actualFrames = dataTable(n+1,9);
    predictedFrames = motionFramePredictionModelFullStartPos.predictFcn(dataTable(n+1,1:8));
    
    actualMotion = [x(startFrame:stopFrame,3) y(startFrame:stopFrame,3) z(startFrame:stopFrame,3)];
    xSpace = (0:actualFrames)/predictedFrames;        
    
    figure(1)
    plot(1:stopFrame-startFrame+1,x(startFrame:stopFrame,3),'r');
    hold on;
    cf_x_pos.A = minX(n+1);
    cf_x_pos.B = LRPredictedLogisticDataTau(n,1);
    cf_x_pos.C = LRPredictedLogisticDataTau(n,2);
    cf_x_pos.D = maxX(n+1);
    ySpace = cf_x_pos(xSpace);
    plot(0:actualFrames,ySpace,'m');
    motionErrX = abs(actualMotion(1:actualFrames+1,1) - ySpace);
    
    plot(1:stopFrame-startFrame+1,y(startFrame:stopFrame,3),'g');
    cf_y_pos.A = minY(n+1);
    cf_y_pos.B = LRPredictedLogisticDataTau(n,3);
    cf_y_pos.C = LRPredictedLogisticDataTau(n,4);
    cf_y_pos.D = maxY(n+1);
    ySpace = cf_y_pos(xSpace);
    plot(0:actualFrames,ySpace,'y');
    motionErrY = abs(actualMotion(1:actualFrames+1,2) - ySpace);
    
    plot(1:stopFrame-startFrame+1,z(startFrame:stopFrame,3),'b');
    cf_z_pos.A = minZ(n+1);
    cf_z_pos.B = LRPredictedLogisticDataTau(n,5);
    cf_z_pos.C = LRPredictedLogisticDataTau(n,6);
    cf_z_pos.D = maxZ(n+1);
    ySpace = cf_z_pos(xSpace);
    plot(0:actualFrames,ySpace,'c');
    motionErrZ = abs(actualMotion(1:actualFrames+1,3) - ySpace);
    hold off;
    
    for vec=1:1:size(motionErrZ,1)
       motionErrAbs(vec) = ((motionErrX(vec)^2)+(motionErrY(vec)^2)+(motionErrZ(vec)^2))^.5; 
    end
    motionErrX = abs(motionErrX);
    motionErrY = abs(motionErrY);
    motionErrZ = abs(motionErrZ);
    
    predictionError(n,1) = max(motionErrX);
    predictionError(n,2) = max(motionErrY);
    predictionError(n,3) = max(motionErrZ);
    predictionError(n,4) = max(motionErrAbs);
    
    quickScript; 
   
    xSpace = transpose(1/(stopFrame-startFrame+1):1/(stopFrame-startFrame+1):1);

    %%{
    endEarlyMotion = int16((endEarlyMotion/100)/(1/(stopFrame-startFrame+1)));
    
    
    ySpace = sampled_cf_j1(xSpace);
    err_sampled(n,1) = mean(abs(transpose(joint_angles(1,startFrame:stopFrame)) - ySpace));
    ySpace = sampled_cf_j2(xSpace);
    err_sampled(n,2) = mean(abs(transpose(joint_angles(2,startFrame:stopFrame)) - ySpace));
    ySpace = sampled_cf_j3(xSpace);
    err_sampled(n,3) = mean(abs(transpose(joint_angles(3,startFrame+endEarlyMotion:stopFrame)) - ySpace(1+endEarlyMotion:stopFrame-startFrame+1)));
    ySpace = sampled_cf_j4(xSpace);
    err_sampled(n,4) = mean(abs(transpose(joint_angles(4,startFrame:stopFrame)) - ySpace));
    ySpace = sampled_cf_j5(xSpace);
    err_sampled(n,5) = mean(abs(transpose(joint_angles(5,startFrame:stopFrame)) - ySpace));
    
    %predictionError(n) = actualFrames-predictedFrames;
    %}
end
%err_sampled_degrees = err_sampled *180/pi;

