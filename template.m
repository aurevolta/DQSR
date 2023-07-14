function [sys]=onesat_S(x,dx,xcg,vcg,time,II,ext_forces,~,gravity_fun)
% template for one single satellite


% Author: Aureliano Rivolta
% e-mail: aureliano.rivolta@polimi.it
% year: 2016
% private use only


%% system gen

n_bodies=1;
n_var=length(x);

% (quaternion norm satisfied on accel level. normalization after is better to have anyway)
% C_phi=q01'*q01-1;
C_C=x(1:4,1)';
C_dC=dx(1:4,1)';


sys=systemS(n_bodies,x(1:4),dx(1:4),xcg(:,1),vcg(:,1),time,gravity_fun,C_C,C_dC);


%% body
B1=rigidBASEbodyS(II(:,1),x(1:4,1),dx(1:4,1),n_var);


%% forces
B1.body_f=[ext_forces(4:6);ext_forces(1:3)];
B1.body_f_loc=zeros(3,1);

%% add body to system
sys=add_body(sys,1,B1);


end
