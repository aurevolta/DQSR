function [rj,drj,J1,J2]=rotational_joint2(rj_axis,Theta,dTheta,d1,d2)
% this function computes the dq state and jacobians of a rotational joint
% given the angle and its velocity and the axis of rotation.
% d1 is the displacement of the joint in the previous reference frame, d2
% is the displacement in the new reference frame. 
% for the particular case of rotational joint J = Jr = J1 and dJ = Jv =J2

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

if nargin == 3
    % no displacememts
    T=zeros(4);
elseif nargin ==4
    % compute the displacement matrix
    T=(crossqp([d1/2;0]));
else
    % compute the displacement matrix
    T=(crossqp([d1/2;0])+crossqm([d2/2;0]));
end

% define the joint rotation dq
q_l=[rj_axis*sin(Theta/2);cos(Theta/2)];
t_l=T*q_l;

% define the derivatives (1st and 2nd)
dqdth=.5*[rj_axis*cos(Theta/2);-sin(Theta/2)];
ddqddth=.25*[-rj_axis*sin(Theta/2);-cos(Theta/2)];

% define the joint velocity dq
dq_l=dqdth*dTheta;
dt_l=T*dq_l;

% define jacobians
J1=[dqdth;T*dqdth];
J2=[ddqddth*dTheta;T*ddqddth*dTheta];

% group the output state
rj=[q_l;t_l];
drj=[dq_l;dt_l];

end