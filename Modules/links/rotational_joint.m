function [rj,drj,J1,J2]=rotational_joint(rj_axis,Theta,dTheta,linkv)
% this function computes the dq state and jacobians of a rotational joint
% given the angle and its velocity and the axis of rotation.
% If a fixed displacement is considered, then it also includes that.
% for the particular case of rotational joint J = Jr = J1 and dJ = Jv =J2
% OLD version

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% in case of no (fixed) displacement
if nargin <4
    
    % define the joint rotation dq
    q_l=[rj_axis*sin(Theta/2);cos(Theta/2)];
    t_l=zeros(4,1);
    
    % define the derivatives (1st and 2nd)
    dqdth=.5*[rj_axis*cos(Theta/2);-sin(Theta/2)];
    ddqddth=.25*[-rj_axis*sin(Theta/2);-cos(Theta/2)];
    
    % define the joint velocity dq
    dq_l=dqdth*dTheta;
    dt_l=zeros(4,1);
    
    % define jacobians
    
    % J=da/dphi
    J1=[dqdth;zeros(4,1)];
    % dJ=d/dt(J)
    J2=[ddqddth*dTheta;zeros(4,1)];
    % Jv=da/phi = J2
    % Jv=[ddqddth*dTheta;zeros(4,1)];
    % Jr=a/phi = J1
    % Jr=[dqdth;zeros(4,1)];
    
else
    
    % define the joint rotation dq
    q_l=[rj_axis*sin(Theta/2);cos(Theta/2)];
    t_l=crossqp([linkv/2;0])*q_l;
    
    % define the derivatives (1st and 2nd)
    dqdth=.5*[rj_axis*cos(Theta/2);-sin(Theta/2)];
    ddqddth=.25*[-rj_axis*sin(Theta/2);-cos(Theta/2)];
    
    % define the joint velocity dq
    dq_l=dqdth*dTheta;
    dt_l=crossqp([linkv/2;0])*dq_l;
    
    % define jacobians
    
    % J=da/dphi
    J1=[dqdth;crossqp([linkv/2;0])*dqdth];
    % dJ=d/dt(J)
    J2=[ddqddth*dTheta;crossqp([linkv/2;0])*ddqddth*dTheta];
    % Jv=da/phi
    % Jv=[ddqddth*dTheta;crossqp([linkv/2;0])*ddqddth*dTheta];
    % Jr=a/phi
    % Jr=[dqdth;crossqp([linkv/2;0])*dqdth];
    
end

% group the output state
rj=[q_l;t_l];
drj=[dq_l;dt_l];

end