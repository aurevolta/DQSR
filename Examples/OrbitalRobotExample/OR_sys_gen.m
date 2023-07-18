function [sys]=OR_sys_gen(x,dx,xcg,vcg,time,II,ext_forces,xi,gravity_fun)
% orbital robot sys gen function


% system size
n_joints = 7;
n_bodies = 1 + n_joints;
n_var = length(x);

% state 
q0 = x(1:4,1);
q0 = q0./norm(q0);
dq0 = dx(1:4,1);
Theta = x(5:11,1);
dTheta = dx(5:11,1);

% constraints
C_phi = (q0'*q0-1);
C_C = [q0', zeros(1,n_joints)];
C_dC = [dq0', zeros(1,n_joints)];


% create data structure
sys = systemS(n_bodies,x,dx,xcg(:,1),vcg(:,1),time,gravity_fun,C_C,C_dC);

% robotic arm
[~,rj_axis,l_cg,b_cg,dqs0] = OR_robotic_arm_parameters;
DQS0 = [dqs0,repmat(zdq,1,6)];
arm_Torque = ext_forces(7:13);

%% complete the system

% base
B1 = rigidBASEbodyS(II(:,1),q0,dq0,n_var);
B1.body_f=[ext_forces(4:6);ext_forces(1:3)];
B1.body_f_loc=zeros(3,1);

sys = add_body(sys,1,B1);

% initialize chain of bodies
B = B1;

for j = 2 : n_bodies
    B = rigidLINKbodyS(II(:,j),B,DQS0(:,j-1),n_var,3+j,'R',l_cg(:,j-1),...
        b_cg(:,j-1),rj_axis(:,j-1),Theta(j-1),dTheta(j-1),arm_Torque(j-1));
    sys = add_body(sys,j,B);
end

end



