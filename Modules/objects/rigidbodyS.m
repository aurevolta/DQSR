function body=rigidbodyS(II,a,da,J,dJ,nvar)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% properties
body.ISRIGID=1; % rigid
body.ISCONSTMASS=1; %constant mass
body.ISGHOST=1; % no contact possible

% mass properties
body.mass=II(4);
body.inertia=II(1:3);

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

% internal forces
body.inner_f=zeros(nvar,1);

% GC projected forces
body.Fcg=zeros(3,3); % projected forces on system cg (body,inertial,internal)

% projected forces
body.proj_f=zeros(nvar,3);

%% body physical dimensions

% maximum spherical radius (sphere of influence)
body.shape.R=sqrt(3);

% 3D points composing the outer shell (default cube unitary side)
body.shape.P=[0.5000    0.5000    0.5000   -0.5000   -0.5000   -0.5000   -0.5000    0.5000
              0.5000   -0.5000    0.5000    0.5000   -0.5000   -0.5000    0.5000   -0.5000
              0.5000    0.5000   -0.5000    0.5000    0.5000   -0.5000   -0.5000   -0.5000];

% [n x 3] matrix of point index connected to make a triangular surface (default cube unitary side)
body.shape.faces=[5     1     3     8     1     4     5     7     5     1     1     6     8
                  4     8     1     5     2     7     8     6     4     2     4     7     3
                  2     3     4     2     8     3     6     5     7     5     5     8     7]';

end