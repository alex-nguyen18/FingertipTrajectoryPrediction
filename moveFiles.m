%%{
LLTestData = zeros(1,15);
LRTestData = zeros(1,15);
ULTestData = zeros(1,15);
URTestData = zeros(1,15);
for n=1:1:50
    %left
    if dataTable(n,1)>0
        if dataTable(n,2)>0
            copyfile(['TestData\',file1(n).name],'ULTest');
            ULTestData = cat(1,ULTestData,[dataTable(n,:) TestDataResultsS1(n,:)]);           
            %ULTestData = cat(1,ULTestData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
        else
            copyfile(['TestData\',file1(n).name],'LLTest');
            LLTestData = cat(1,LLTestData,[dataTable(n,:) TestDataResultsS1(n,:)]);           
            %LLTestData = cat(1,LLTestData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
        end
    %right
    else
        if dataTable(n,2)>0
            copyfile(['TestData\',file1(n).name],'URTest');
            URTestData = cat(1,URTestData,[dataTable(n,:) TestDataResultsS1(n,:)]);           
            %URTestData = cat(1,URTestData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
        else 
            copyfile(['TestData\',file1(n).name],'LRTest');
            LRTestData = cat(1,LRTestData,[dataTable(n,:) TestDataResultsS1(n,:)]);           
            %LRTestData = cat(1,LRTestData,[dataTable(n,:) xPosLogisticData(n,2:3) yPosLogisticData(n,2:3) zPosLogisticData(n,2:3)]);
        end
    end
end
%}

