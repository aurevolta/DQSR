function [ext_f,Fcg]=gravity_model0(sys,a,J,xcg,I,n_bodies)
% compute force and NO torque due to gravity (single spherical main attractor)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

if isempty(sys.gravity_parameters)
    % gravity constant (Earth)
    mu=3.986*10^5*1e9;
else
    mu=sys.gravity_parameters;
end


% initialize
[Force,Torque]=deal(zeros(3,n_bodies));

% for all bodies
for i=1:n_bodies
    
    % position
    CMt=crossqm(a(1:4,i))';
    rcgi=xcg+2*CMt(1:3,:)*a(5:8,i);
    RCGI=norm(rcgi);
    
    % compute forces and torques on body i
    Force(:,i)=-mu*I(4,i)/(RCGI^3)*rcgi;
    % already in inertial reference frame
end

% generalized forces for lagrange eq.
ext_f=gfa(a,J,Force,Torque,n_bodies);

% compute gravity forces acting on cg
Fcg=sum(Force,2);

end




