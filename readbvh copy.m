%%读取bvh文件，从hip开始到手指
%%只有位置，没有姿态。
%%查看bvh文件，找到以上各部位所对应的offset

function [p_elbow,p_wrist,p_hand,r_elbow,r_wrist,r_hand]=readbvh(offset,filename)
hips_chest=offset(1,:)';
chest_rightcollar=offset(2,:)';
rightcollar_rightshoulder=offset(3,:)';
rightshoulder_rightelbow=offset(4,:)';
rightelbow_rightwrist=offset(5,:)';
rightwrist_righthand=offset(6,:)';
R01=[-1 0 0;0 -1 0;0 0 1];%DH坐标系下的基系
Rdh30=[0 -1 0;1 0 0;0 0 1];%初始时刻DH坐标系中的3系与bvh中的基系的转换关系
Rdh50=[0 -1 0;1 0 0;0 0 1];%初始时刻DH坐标系中的5系与bvh中的基系的转换关系
Rdh70=[-1 0 0;0 0 1;0 1 0];%初始时刻DH坐标系中的7系与bvh中的基系的转换关系


%%rightcollar在bvh中为第九个节点，通道为28-30；其他以此类推。hand在bvh中没有信息,提取有用通道，提取运动中间部分
data=dlmread(filename,' ',114,0);
data=data(:,1:4:225);
datause=data(:,[1:9,28:39])*pi/180;
K=size(datause,1);
M=5;  %%间隔帧数
loop=1; %%循环计数
for i=1:M:K
p_hips(loop,:)=data(i,1:3);
    
Rz12=[cos(datause(i,4)) -sin(datause(i,4)) 0;sin(datause(i,4)) cos(datause(i,4)) 0;0 0 1];
Rx12=[1 0 0;0 cos(datause(i,5)) -sin(datause(i,5));0 sin(datause(i,5)) cos(datause(i,5))];
Ry12=[cos(datause(i,6)) 0 sin(datause(i,6));0 1 0;-sin(datause(i,6)) 0 cos(datause(i,6))];
R12=Rz12*Rx12*Ry12;
p_chest(loop,:)=(R01*R12*hips_chest)'+p_hips(loop,:);

Rz23=[cos(datause(i,7)) -sin(datause(i,7)) 0;sin(datause(i,7)) cos(datause(i,7)) 0;0 0 1];
Rx23=[1 0 0;0 cos(datause(i,8)) -sin(datause(i,8));0 sin(datause(i,8)) cos(datause(i,8))];
Ry23=[cos(datause(i,9)) 0 sin(datause(i,9));0 1 0;-sin(datause(i,9)) 0 cos(datause(i,9))];
R23=Rz23*Rx23*Ry23;
p_rightcollar(loop,:)=(R01*R12*R23*chest_rightcollar+p_chest(loop,:)')';

Rz34=[cos(datause(i,10)) -sin(datause(i,10)) 0;sin(datause(i,10)) cos(datause(i,10)) 0;0 0 1];
Rx34=[1 0 0;0 cos(datause(i,11)) -sin(datause(i,11));0 sin(datause(i,11)) cos(datause(i,11))];
Ry34=[cos(datause(i,12)) 0 sin(datause(i,12));0 1 0;-sin(datause(i,12)) 0 cos(datause(i,12))];
R34=Rz34*Rx34*Ry34;
p_rightshoulder(loop,:)=(R01*R12*R23*R34*rightcollar_rightshoulder+p_rightcollar(loop,:)')';

Rz45=[cos(datause(i,13)) -sin(datause(i,13)) 0;sin(datause(i,13)) cos(datause(i,13)) 0;0 0 1];
Rx45=[1 0 0;0 cos(datause(i,14)) -sin(datause(i,14));0 sin(datause(i,14)) cos(datause(i,14))];
Ry45=[cos(datause(i,15)) 0 sin(datause(i,15));0 1 0;-sin(datause(i,15)) 0 cos(datause(i,15))];
R45=Rz45*Rx45*Ry45;
p_rightelbow(loop,:)=(R01*R12*R23*R34*R45*rightshoulder_rightelbow+p_rightshoulder(loop,:)')';
r_elbow(3*loop-2:3*loop,:)=R01*R12*R23*R34*R45*Rdh30;

Rz56=[cos(datause(i,16)) -sin(datause(i,16)) 0;sin(datause(i,16)) cos(datause(i,16)) 0;0 0 1];
Rx56=[1 0 0;0 cos(datause(i,17)) -sin(datause(i,17));0 sin(datause(i,17)) cos(datause(i,17))];
Ry56=[cos(datause(i,18)) 0 sin(datause(i,18));0 1 0;-sin(datause(i,18)) 0 cos(datause(i,18))];
R56=Rz56*Rx56*Ry56;
p_rightwrist(loop,:)=(R01*R12*R23*R34*R45*R56*rightelbow_rightwrist+p_rightelbow(loop,:)')';
r_wrist(3*loop-2:3*loop,:)=R01*R12*R23*R34*R45*R56*Rdh50;

Rz67=[cos(datause(i,19)) -sin(datause(i,19)) 0;sin(datause(i,19)) cos(datause(i,19)) 0;0 0 1];
Rx67=[1 0 0;0 cos(datause(i,20)) -sin(datause(i,20));0 sin(datause(i,20)) cos(datause(i,20))];
Ry67=[cos(datause(i,21)) 0 sin(datause(i,21));0 1 0;-sin(datause(i,21)) 0 cos(datause(i,21))];
R67=Rz67*Rx67*Ry67;
p_righthand(loop,:)=(R01*R12*R23*R34*R45*R56*R67*rightwrist_righthand+p_rightwrist(loop,:)');
r_hand(3*loop-2:3*loop,:)=R01*R12*R23*R34*R45*R56*R67*Rdh70;

x=[p_hips(loop,1),p_chest(loop,1),p_rightcollar(loop,1),p_rightshoulder(loop,1),p_rightelbow(loop,1),p_rightwrist(loop,1),p_righthand(loop,1)];
y=[p_hips(loop,2),p_chest(loop,2),p_rightcollar(loop,2),p_rightshoulder(loop,2),p_rightelbow(loop,2),p_rightwrist(loop,2),p_righthand(loop,2)];
z=[p_hips(loop,3),p_chest(loop,3),p_rightcollar(loop,3),p_rightshoulder(loop,3),p_rightelbow(loop,3),p_rightwrist(loop,3),p_righthand(loop,3)];
plot3(-x,-z,-y,'linewidth',1.5)
%plot3(-p_righthand(loop,1),-p_righthand(loop,3),-p_righthand(loop,2),'.','linewidth',1.5);
hold on
axis equal
loop=loop+1;
end

stdx=std(-p_righthand(:,1))
stdy=std(-p_righthand(:,3))

%%%%%%%%%%%%%%%%%%%
%%每个时刻把shoulder看成原点，求出肘部腕部手部在肩部坐标的位置。
N=size(p_chest,1);
p_elbow=p_rightelbow-p_rightshoulder;
p_wrist=p_rightwrist-p_rightshoulder;
p_hand=p_righthand-p_rightshoulder;
figure(2)
for i=1:N;
x=[0,p_elbow(i,1),p_wrist(i,1),p_hand(i,1)];
y=[0,p_elbow(i,2),p_wrist(i,2),p_hand(i,2)];
z=[0,p_elbow(i,3),p_wrist(i,3),p_hand(i,3)];
plot3(-x,-z,-y,'linewidth',1.5);
hold on
axis equal
end


