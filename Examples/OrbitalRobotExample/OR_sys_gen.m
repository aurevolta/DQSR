function [a,da,J,dJ,C_phi,C_C,C_dC,ext_f,n_bodies,Fcg]=OR_sys_gen(x,dx,xcg,vcg,time,II,ext_forces)
% orbital robot sys gen function

mu=3.986*10^5*1e9; %km^3/s^2

% state
q0=x(1:4,1);
dq0=dx(1:4,1);
Theta=x(5:11,1);
dTheta=dx(5:11,1);
q0=q0./norm(q0);


% load important geometrical data
[~,rj_axis,l_cg,b_cg,dqs0]=robotic_arm_parameters;


n_bodies=8;
n_joints=7;

%% computation

setz=['R','R','R','R','R','R','R'];

[a0,da0,J0,dJ0]=robotic_arm([q0;zeros(4,1)],[dq0;zeros(4,1)],rj_axis,Theta,dTheta,setz,l_cg,b_cg,dqs0,4,[eye(4);zeros(4)]);
[a,da,J,dJ]=enforce_cg(a0,da0,J0,dJ0,II(end,:));

%% forces

% base attitude forces and torques
R4=crossqp(q0)*(crossqm(q0)');
R=R4(1:3,1:3);
ext_f=gfa(a(:,1),J(:,:,1),R*ext_forces(4:6),R*ext_forces(1:3),1);

% CG forces = gravity + control forces
Fcg=-mu/(norm(xcg)^3)*xcg*sum(II(end,:))+R*ext_forces(4:6);


% control torques (inserted directly on the active dofs)
control_forces=ext_forces(7:(6+n_joints),1);
ext_f=ext_f+[zeros(4,1);control_forces];

%% constraints

C_phi=(q0'*q0-1);
C_C=[q0',zeros(1,n_joints)];
C_dC=[dq0',zeros(1,n_joints)];

end



function [lengths,rj_axis,l_cg,b_cg,dqs0]=robotic_arm_parameters

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