%LLPredictedLogisticDataTau=[LLHillsSlopeXModelTau.predictFcn(LLTestData(:,1:10)) LLInflectionXModelTau.predictFcn(LLTestData(:,1:10)) LLHillsSlopeYModelTau.predictFcn(LLTestData(:,1:10)) LLInflectionYModelTau.predictFcn(LLTestData(:,1:10)) LLHillsSlopeZModelTau.predictFcn(LLTestData(:,1:10)) LLInflectionZModelTau.predictFcn(LLTestData(:,1:10))];
%LRPredictedLogisticDataTau=[LRHillsSlopeXModelTau.predictFcn(LRTestData(:,1:10)) LRInflectionXModelTau.predictFcn(LRTestData(:,1:10)) LRHillsSlopeYModelTau.predictFcn(LRTestData(:,1:10)) LRInflectionYModelTau.predictFcn(LRTestData(:,1:10)) LRHillsSlopeZModelTau.predictFcn(LRTestData(:,1:10)) LRInflectionZModelTau.predictFcn(LRTestData(:,1:10))];
%ULPredictedLogisticDataTau=[ULHillsSlopeXModelTau.predictFcn(ULTestData(:,1:10)) ULInflectionXModelTau.predictFcn(ULTestData(:,1:10)) ULHillsSlopeYModelTau.predictFcn(ULTestData(:,1:10)) ULInflectionYModelTau.predictFcn(ULTestData(:,1:10)) ULHillsSlopeZModelTau.predictFcn(ULTestData(:,1:10)) ULInflectionZModelTau.predictFcn(ULTestData(:,1:10))];
%URPredictedLogisticDataTau=[URHillsSlopeXModelTau.predictFcn(URTestData(:,1:10)) URInflectionXModelTau.predictFcn(URTestData(:,1:10)) URHillsSlopeYModelTau.predictFcn(URTestData(:,1:10)) URInflectionYModelTau.predictFcn(URTestData(:,1:10)) URHillsSlopeZModelTau.predictFcn(URTestData(:,1:10)) URInflectionZModelTau.predictFcn(URTestData(:,1:10))];
%{
tauXInflectionLL = LLTrainData(:,13) ./ transpose(motionFramesLLTrain);
tauYInflectionLL = LLTrainData(:,15) ./ transpose(motionFramesLLTrain);
tauZInflectionLL = LLTrainData(:,17) ./ transpose(motionFramesLLTrain);

tauXInflectionLR = LRTrainData(:,13) ./ transpose(motionFramesLRTrain);
tauYInflectionLR = LRTrainData(:,15) ./ transpose(motionFramesLRTrain);
tauZInflectionLR = LRTrainData(:,17) ./ transpose(motionFramesLRTrain);

tauXInflectionUL = ULTrainData(:,13) ./ transpose(motionFramesULTrain);
tauYInflectionUL = ULTrainData(:,15) ./ transpose(motionFramesULTrain);
tauZInflectionUL = ULTrainData(:,17) ./ transpose(motionFramesULTrain);

tauXInflectionUR = URTrainData(:,13) ./ transpose(motionFramesURTrain);
tauYInflectionUR = URTrainData(:,15) ./ transpose(motionFramesURTrain);
tauZInflectionUR = URTrainData(:,17) ./ transpose(motionFramesURTrain);
%}
%{
LLFTestData = zeros(1,12);
LRFTestData = zeros(1,12);
ULFTestData = zeros(1,12);
URFTestData = zeros(1,12);
LLBTestData = zeros(1,12);
LRBTestData = zeros(1,12);
ULBTestData = zeros(1,12);
URBTestData = zeros(1,12);

LLFTrainData = zeros(1,18);
LRFTrainData = zeros(1,18);
ULFTrainData = zeros(1,18);
URFTrainData = zeros(1,18);
LLBTrainData = zeros(1,18);
LRBTrainData = zeros(1,18);
ULBTrainData = zeros(1,18);
URBTrainData = zeros(1,18);
dataTable = trainingData;
for n=1:1:size(file,1)
    if(trainingData(n,3)>=(trainingData(n,8)+trainingData(n,9))/2)
        if(trainingData(n,2)>=0)
            if(trainingData(n,1)>=0)
                copyfile(['TrainingData\',file(n).name],'ULFTrain\');
                ULFTrainData = cat(1,ULFTrainData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
            else
                copyfile(['TrainingData\',file(n).name],'URFTrain\');
                URFTrainData = cat(1,URFTrainData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
            end
        else
            if(trainingData(n,1)>=0)
                copyfile(['TrainingData\',file(n).name],'LLFTrain\');
                LLFTrainData = cat(1,LLFTrainData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
            else
                copyfile(['TrainingData\',file(n).name],'LRFTrain\');
                LRFTrainData = cat(1,LRFTrainData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
            end
        end
    else
        if(trainingData(n,2)>=0)
            if(trainingData(n,1)>=0)
                copyfile(['TrainingData\',file(n).name],'ULBTrain\');
                ULBTrainData = cat(1,ULBTrainData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
            else
                copyfile(['TrainingData\',file(n).name],'URBTrain\');
                URBTrainData = cat(1,URBTrainData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
            end
        else
            if(trainingData(n,1)>=0)
                copyfile(['TrainingData\',file(n).name],'LLBTrain');
                LLBTrainData = cat(1,LLBTrainData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
            else
                copyfile(['TrainingData\',file(n).name],'LRBTrain');
                LRBTrainData = cat(1,LRBTrainData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
            end
        end        
    end
end
%}
%{
dataTable = testData;
for n=1:1:size(file1,1)
    if(testData(n,3)>=(testData(n,8)+testData(n,9))/2)
        if(testData(n,2)>=0)
            if(testData(n,1)>=0)
                copyfile(['testData\',file1(n).name],'ULFTest');
                ULFTestData = cat(1,ULFTestData,[dataTable(n,:)]);
            else
                copyfile(['testData\',file1(n).name],'URFTest');
                URFTestData = cat(1,URFTestData,[dataTable(n,:)]);
            end
        else
            if(testData(n,1)>=0)
                copyfile(['testData\',file1(n).name],'LLFTest');
                LLFTestData = cat(1,LLFTestData,[dataTable(n,:)]);
            else
                copyfile(['testData\',file1(n).name],'LRFTest');
                LRFTestData = cat(1,LRFTestData,[dataTable(n,:)]);
            end
        end
    else
        if(testData(n,2)>=0)
            if(testData(n,1)>=0)
                copyfile(['testData\',file1(n).name],'ULBTest');
                ULBTestData = cat(1,RLBTestData,[dataTable(n,:)]);
            else
                copyfile(['testData\',file1(n).name],'URBTest');
                URBTestData = cat(1,URBTestData,[dataTable(n,:)]);
            end
        else
            if(testData(n,1)>=0)
                copyfile(['testData\',file1(n).name],'LLBTest');
                LLBTestData = cat(1,LLBTestData,[dataTable(n,:)]);
            else
                copyfile(['testData\',file1(n).name],'LRBTest');
                LRBTestData = cat(1,LRBTestData,[dataTable(n,:)]);
            end
        end        
    end
end
%}
%%{
sampledPoints = [cf_x_pos([transpose(.01:.01:1)]) cf_y_pos([transpose(.01:.01:1)]) cf_z_pos([transpose(.01:.01:1)])];
sampledJoints = zeros(1,7);
%%{
for p=1:100
   rWristSSample = norm(sampledPoints(p,:));
   rWristCSample = (sampledPoints(p,1)^2+sampledPoints(p,3)^2)^.5;
   thetaWristSample = acos(sampledPoints(p,1)/rWristCSample);
   phiWristSample = acos(rWristCSample/rWristSSample);
   sampleTable(p,:) = [sampledPoints(p,1) sampledPoints(p,2) sampledPoints(p,3) rWristSSample phiWristSample thetaWristSample normOFFSET(1) normOFFSET(2) (normOFFSET(1)+normOFFSET(2)) rWristSSample/(normOFFSET(1)+normOFFSET(2)) 0];
   if p<=20
       sampleTable(p,11) = ESAModel20Atan.predictFcn(sampleTable(p,1:10));
   elseif p<=40
       sampleTable(p,11) = ESAModel40Atan.predictFcn(sampleTable(p,1:10));
   elseif p<=60
       sampleTable(p,11) = ESAModel60Atan.predictFcn(sampleTable(p,1:10));
   elseif p<=80
       sampleTable(p,11) = ESAModel80Atan.predictFcn(sampleTable(p,1:10));
   else
       sampleTable(p,11) = ESAModel100Atan.predictFcn(sampleTable(p,1:10));
   end
end
for p=1:100
    if norm(sampledPoints(p,:))>=(normOFFSET(1)+normOFFSET(2))
        continue
    end
    sampledJoints=cat(1,sampledJoints,[inverse_swivel(sampleTable(p,11),normOFFSET,transpose(sampledPoints(p,:)),.01*p) p]);

end
sampledJoints = sampledJoints(2:size(sampledJoints,1),:);
for p=1:size(sampledJoints,1)
    if sampledJoints(p,6) == 1 && sampledJoints(p,7)<=50
        endEarlyMotion = sampledJoints(p,7);
    end
end
%}
%%{
%for p=1:size(sampledJoints,1);
%   sampledForwardKin(p,:) = [(cos(sampledJoints(p,1))*sin(sampledJoints(p,2))*cos(sampledJoints(p,3))-sin(sampledJoints(p,1))*sin(sampledJoints(p,3)))*sin(sampledJoints(p,4))*normOFFSET(2)-cos(sampledJoints(p,1))*cos(sampledJoints(p,2))*cos(sampledJoints(p,4))*normOFFSET(2)-cos(sampledJoints(p,1))*cos(sampledJoints(p,2))*normOFFSET(1) (sin(sampledJoints(p,1))*sin(sampledJoints(p,2))*cos(sampledJoints(p,3))-cos(sampledJoints(p,1))*sin(sampledJoints(p,3)))*sin(sampledJoints(p,4))*normOFFSET(2)-sin(sampledJoints(p,1))*cos(sampledJoints(p,2))*cos(sampledJoints(p,4))*normOFFSET(2)-sin(sampledJoints(p,1))*cos(sampledJoints(p,2))*normOFFSET(1) sin(sampledJoints(p,4))*cos(sampledJoints(p,2))*cos(sampledJoints(p,3))*normOFFSET(2)+normOFFSET(2)*sin(sampledJoints(p,2))*cos(sampledJoints(p,4))+sin(sampledJoints(p,2))*normOFFSET(1)];
%end
    [sampled_cf_j1,sampled_G_j1_pos] = L5P(sampledJoints(:,7)/100,sampledJoints(:,1));
    [sampled_cf_j2,sampled_G_j2_pos] = L5P(sampledJoints(:,7)/100,sampledJoints(:,2));
    [sampled_cf_j3,sampled_G_j3_pos] = L5P(sampledJoints(:,7)/100,sampledJoints(:,3));
    [sampled_cf_j4,sampled_G_j4_pos] = L5P(sampledJoints(:,7)/100,sampledJoints(:,4));
    [sampled_cf_j5,sampled_G_j5_pos] = L5P(sampledJoints(:,7)/100,sampledJoints(:,5));
    
    %figure(1);
   
    %{
    xSampleSpace = .01:.01:1;
   
    freezeFrames = [0;.25;.5;.75;1];
    freezeAxis = zeros(1,5);
    for freeze=1:1:5
        predicted(:) = [sampled_cf_j1(freezeFrames(freeze)); sampled_cf_j2(freezeFrames(freeze)); sampled_cf_j3(freezeFrames(freeze)); sampled_cf_j4(freezeFrames(freeze))];
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
    freezeAxis = cat(1,freezeAxis,[axis elbowAng]);
    end
    freezeAxis;
    %{
    %Logistic
    ySampleSpace = sampled_cf_j1(xSampleSpace);
    plot(xSampleSpace,ySampleSpace,'m');
    hold on;
    ySampleSpace = sampled_cf_j2(xSampleSpace);
    plot(xSampleSpace,ySampleSpace,'y');
    ySampleSpace = sampled_cf_j3(xSampleSpace);
    plot(xSampleSpace,ySampleSpace,'c');
    ySampleSpace = sampled_cf_j4(xSampleSpace);
    plot(xSampleSpace,ySampleSpace,'r');
    ySampleSpace = sampled_cf_j5(xSampleSpace);
    plot(xSampleSpace,ySampleSpace,'g');
    
    plot(sampledJoints(:,6)/100,sampledJoints(:,1),'r');
    plot(sampledJoints(:,6)/100,sampledJoints(:,2),'g');
    plot(sampledJoints(:,6)/100,sampledJoints(:,3),'b');
    plot(sampledJoints(:,6)/100,sampledJoints(:,4),'m');
    plot(sampledJoints(:,6)/100,sampledJoints(:,5),'y');
    hold off;
    
    figure(2);
    hold off;
        %Logistic
    ySampleSpace = sampled_cf_j1(xSampleSpace);
    plot(xSampleSpace,ySampleSpace,'m');
    hold on;
    ySampleSpace = sampled_cf_j2(xSampleSpace);
    plot(xSampleSpace,ySampleSpace,'y');
    ySampleSpace = sampled_cf_j3(xSampleSpace);
    plot(xSampleSpace,ySampleSpace,'c');
    ySampleSpace = sampled_cf_j4(xSampleSpace);
    plot(xSampleSpace,ySampleSpace,'r');
    ySampleSpace = sampled_cf_j5(xSampleSpace);
    plot(xSampleSpace,ySampleSpace,'g');
    
    plot(xSpace,joint_angles(1,startFrame:stopFrame),'r');
    plot(xSpace,joint_angles(2,startFrame:stopFrame),'g');
    plot(xSpace,joint_angles(3,startFrame:stopFrame),'b');
    hold on;
    plot(xSpace,joint_angles(4,startFrame:stopFrame),'m');
    plot(xSpace,joint_angles(5,startFrame:stopFrame),'y');
    plot(xSpace,joint_angles(6,startFrame:stopFrame),'c');
    
    hold off;
    %}
    %}
%{
for p=1:stopFrame-startFrame+1
   bvhForwardKin(p,:) = [(cos(bvhJoints(p,1))*sin(bvhJoints(p,2))*cos(bvhJoints(p,3))-sin(bvhJoints(p,1))*sin(bvhJoints(p,3)))*sin(bvhJoints(p,4))*normOFFSET(2)-cos(bvhJoints(p,1))*cos(bvhJoints(p,2))*cos(bvhJoints(p,4))*normOFFSET(2)-cos(bvhJoints(p,1))*cos(bvhJoints(p,2))*normOFFSET(1) (sin(bvhJoints(p,1))*sin(bvhJoints(p,2))*cos(bvhJoints(p,3))-cos(bvhJoints(p,1))*sin(bvhJoints(p,3)))*sin(bvhJoints(p,4))*normOFFSET(2)-sin(bvhJoints(p,1))*cos(bvhJoints(p,2))*cos(bvhJoints(p,4))*normOFFSET(2)-sin(bvhJoints(p,1))*cos(bvhJoints(p,2))*normOFFSET(1) sin(bvhJoints(p,4))*cos(bvhJoints(p,2))*cos(bvhJoints(p,3))*normOFFSET(2)+normOFFSET(2)*sin(bvhJoints(p,2))*cos(bvhJoints(p,4))+sin(bvhJoints(p,2))*normOFFSET(1)]
end
%}
%}
%{
dataTable20 = zeros(1,12);
dataTable40 = zeros(1,12);
dataTable60 = zeros(1,12);
dataTable80 = zeros(1,12);
dataTable100 = zeros(1,12);
for n=1:size(dataTable,1)
   if dataTable(n,12)<.2
       dataTable20 = cat(1,dataTable20,dataTable(n,:));
   elseif dataTable(n,12)<.4
       dataTable40 = cat(1,dataTable40,dataTable(n,:));
   elseif dataTable(n,12)<.6
       dataTable60 = cat(1,dataTable60,dataTable(n,:));
   elseif dataTable(n,12)<.8
       dataTable80 = cat(1,dataTable80,dataTable(n,:));
   else
       dataTable100 = cat(1,dataTable100,dataTable(n,:));
   end
end
%}