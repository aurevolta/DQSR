function [rj,drj,J1,J2]=translational_joint(rj_axis,x,v,x0)
% this function computes the dq state and jacobians of a translational joint
% given the angle and its velocity and the axis of rotation.
% If a fixed displacement is considered, then it also includes that.
% for the particular case of rotational joint J = Jr = J1 and dJ = Jv =J2

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

q_l=[0;0;0;1];
t_l=.5*[rj_axis*(x+x0);0];
dq_l=zeros(4,1);
dt_l=.5*[rj_axis*v;0];



% J=da/dphi
J1=.5*[zeros(4,1);rj_axis;0];
% dJ=d/dt(J)
J2=.5*zeros(8,1);

% group the output state
rj=[q_l;t_l];
drj=[dq_l;dt_l];

end