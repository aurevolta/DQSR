function [lengths,rj_axis,l_cg,b_cg,dqs0]=OR_robotic_arm_parameters

rx=[1;0;0];ry=[0;1;0];rz=[0;0;1];

% rj_axis=[rz,ry,rz,ry,rz,ry,rz];
rj_axis=[rz,ry,rx,ry,rx,ry,rx];

% link lengths
lengths=[0.1,1,0.2,0.8,0.1,0.1,0.05];

l_cg=([-ry,-rx,ry,rx,-ry,-rx,rx].*([1;1;1]*lengths/2));
b_cg=([-ry,-rx,ry,rx,-ry,-rx,rx].*([1;1;1]*lengths/2));
l_cg=[zeros(3,1),l_cg(:,1:end)];

% dx0=[0.25;0;1/2+0.05];
dx0=[0.75;0;1/2+0.05];
dqs0=[0;0;0;1;dx0/2;0];


end