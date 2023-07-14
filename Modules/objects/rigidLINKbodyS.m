function [body,a2,da2,J2,dJ2]=rigidLINKbodyS(II,basebody,dqs0,nvar,ivar,jointK,xcg,x2,rj_axis,Theta,dTheta,Torque)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% output also the location of the second node (thinking to flex)
% jointK is kind of joint

dqs=dqprodm(dqs0);
a0=basebody.a;
da0=basebody.da;
J0=basebody.J;
dJ0=basebody.dJ;

a0=dqs*a0;
da0=dqs*da0;
J0=dqs*J0;
dJ0=dqs*dJ0;



% properties
body.ISRIGID=1; % rigid
body.ISCONSTMASS=1; %constant mass
body.ISGHOST=1; % no contact possible

% mass properties
body.mass=II(4);
body.inertia=II(1:3);

% node computation
switch jointK
        case 'R'
            [rj,drj,Ji,dJi]=rotational_joint2(rj_axis,Theta,dTheta,zeros(3,1),xcg);
        case 'T'
            [rj,drj,Ji,dJi]=translational_joint(rj_axis,Theta,dTheta,0);
end

RJ=dqprodm(rj);
dRJ=dqprodm(drj);


a=RJ*a0;
da=RJ*da0+dRJ*a0;

J=RJ*J0;
J(:,ivar)=dqprodp(a0)*Ji;
dJ=RJ*dJ0+dRJ*J0;
dJ(:,ivar)=dqprodp(a0)*dJi+dqprodp(da0)*Ji;

% for the node
RJ2=dqprodm([zq;x2/2;0]);
a2=RJ2*a;
da2=RJ2*da;
J2=RJ2*J;
dJ2=RJ2*dJ;


% state and jacobians
body.a=a;
body.da=da;
body.J=J;
body.dJ=dJ;

% number of lagrange variables
body.nvar=nvar;

%% forces

% body forces
body.body_f=zeros(6,1);
body.body_f_loc=zeros(3,1);

% inertial forces
body.inertial_f=zeros(6,1);

% internal forces (add torque)
body.inner_f=zeros(nvar,1);
body.inner_f(ivar)=Torque;

% GC projected forces
body.Fcg=zeros(3,3); % projected forces on system cg (body,inertial,internal)

% projected forces
body.proj_f=zeros(nvar,3);

%% body physical dimensions

% maximum spherical radius (sphere of influence)
body.shape.R=1;

% 3D points composing the outer shell (default cube unitary side)
body.shape.P=[0.5000    0.5000    0.5000   -0.5000   -0.5000   -0.5000   -0.5000    0.5000
              0.5000   -0.5000    0.5000    0.5000   -0.5000   -0.5000    0.5000   -0.5000
              0.5000    0.5000   -0.5000    0.5000    0.5000   -0.5000   -0.5000   -0.5000];

% [n x 3] matrix of point index connected to make a triangular surface (default cube unitary side)
body.shape.faces=[5     1     3     8     1     4     5     7     5     1     1     6     8
                  4     8     1     5     2     7     8     6     4     2     4     7     3
                  2     3     4     2     8     3     6     5     7     5     5     8     7]';

end