function [ext_f,Fcg]=ground_gravity_z(sys,a,J,xcg,I,n_bodies)
% compute force and NO torque due to gravity (single spherical main attractor)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

if isempty(sys.gravity_parameters)
    % gravity constant (Earth)
    g0=9.81;
else
    g0=sys.gravity_parameters;
end

% initialize
[Force,Torque]=deal(zeros(3,n_bodies));

% for all bodies
for i=1:n_bodies
    % compute forces
    Force(:,i)=-[0;0;g0*I(4,i)];
    % already in inertial reference frame
end

% generalized forces for lagrange eq.
ext_f=gfa(a,J,Force,Torque,n_bodies);

% compute gravity forces acting on cg
Fcg=sum(Force,2);

end




