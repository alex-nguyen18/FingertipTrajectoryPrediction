for fr = 1:1:e
    neededJoints(fr,:) = [sampled_cf_j1(fr/e),sampled_cf_j2(fr/e),1*sampled_cf_j3(fr/e),sampled_cf_j4(fr/e)];
end
neededJoints = neededJoints * 180/3.14;
%copyfile 'toBVH\template.BVH' 'toBVH\testMotion.bvh' 
FID = fopen('toBVH\testMotion.bvh','w');
%fseek(FID,0,'eof');
for fr = 1:1:e
    for chan = 1:1:180
        switch (chan)
            case 1
                fprintf(FID, '%3.6f ',OFFSET(1,1));
            case 2
                fprintf(FID, '%3.6f ',OFFSET(1,2));
            case 3
                fprintf(FID, '%3.6f ',OFFSET(1,3));
            case 46
                fprintf(FID, '%3.6f ',neededJoints(fr,3));
            case 47
                fprintf(FID, '%3.6f ',neededJoints(fr,2));
            case 48
                fprintf(FID, '%3.6f ',neededJoints(fr,1));
            case 49
                fprintf(FID, '%3.6f ',neededJoints(fr,4));
            case 117
                fprintf(FID, '-90.000000 '); 
            otherwise
                fprintf(FID, '0.000000 ');
        end
    end
    fprintf(FID, '\n');
end