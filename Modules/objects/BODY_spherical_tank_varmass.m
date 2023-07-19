function [body,dxi]=BODY_spherical_tank_varmass(rho,R,xi,MFR,exhaust_loc,exhaust_dir,Isp,a,da,J,dJ,nvar)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% F = exhaust_dir*Isp*9.81*abs(MFR)

% properties
body.ISRIGID=1; % rigid
body.ISCONSTMASS=0; %constant mass
body.ISGHOST=1; % no contact possible

[m,I,xcg,dI,dxi,MFR]=spherical_tank_parameters(R,rho,xi,MFR);

% mass properties
body.mass=m;
body.inertia=I;

% state and jacobians
DQ=dqprodm([zq;xcg/2;0]);

A=DQ*a;
DA=DQ*da;

body.a=A;
body.da=DA;
body.J=DQ*J;
body.dJ=DQ*dJ;

% number of lagrange variables
body.nvar=nvar;

%% forces

% body forces
omega=2*crossqp(A(1:4,1))'*DA(1:4,1);
body.body_f=[exhaust_dir*Isp*9.81*abs(MFR);+dI.*omega(1:3)];
body.body_f_loc=exhaust_loc-xcg;

% inertial forces
body.inertial_f=zeros(6,1);

% internal forces
body.inner_f=zeros(nvar,1);

% GC projected forces
body.Fcg=zeros(3,3); % projected forces on system cg (body,inertial,internal)

% projected forces
body.proj_f=zeros(nvar,3);



%% body physical dimensions (TO ADJ WITH SPHERE)

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



function [m,I,xcg,dI,dxi,MFR]=spherical_tank_parameters(R,rho,xi,MFR)
% z assumed to be filling direction

if xi <= 1e-6 || xi>=(1-1e-6)
    % if there is no more fuel, no more thrust, mass or inertia
    [m,dxi,MFR]=deal(0);
    [I,dI,xcg]=deal(zeros(3,1));
    return
end

% full tank status
V0 = 4/3 * pi * R^3;
m0 = V0 * rho;
I0=repmat( 8/15 * pi * rho * R^5,3,1);

% adding dependance on xi
m = m0 * (3*xi^2 -2*xi^3);

xcg=[0;
    0;
    R * (4*xi - 3*xi^2)/(3-2*xi)];

I= I0;
I(1:2) = I0(1:2)*(-55*xi^4+36*xi^5-9*xi^6+30)/(6-4*xi);
I(3)   = I0(3)  *(10*xi^3+6*xi^5-15*xi^4);

% filling variation in time
dxi = MFR/(m0*6*(xi-xi^2+eps));

% inertia variation
dIdxi=zeros(3,1);
dIdxi(1:2)=I0(1:2) * ((15*(3*xi^6 - 15*xi^5 + 29*xi^4 - 22*xi^3 + 2))/((2*xi - 3)^2));
dIdxi(3)=I0(3) * (30*xi^4 - 60*xi^3 + 30*xi^2);

dI=dIdxi*dxi;


end





