function [ext_f,Fcg]=gravity_modelCW(sys,a,J,xcg,I,n_bodies)
% compute force and NO torque due to gravity according to CW linearized equations

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

if isempty(sys.gravity_parameters)
    % gravity constant (Earth)
    n=0.0001;
else
    n=sys.gravity_parameters;
end

% initialize
[Force,Torque]=deal(zeros(3,n_bodies));

% for all bodies
for i=1:n_bodies
    
    % position and velocities
    CMt=crossqm(a(1:4,i))';
    DCMt=crossqm(sys.bodies(i).da(1:4))';
    rcgi=xcg+2*CMt(1:3,:)*a(5:8,i);
    vcgi=sys.vcg+2*CMt(1:3,:)*sys.bodies(i).da(5:8)+2*DCMt(1:3,:)*a(5:8,i);
    
    % position
    fi=[3*n(1)^2*rcgi(1)+2*n(1)*vcgi(2);
                        -2*n(1)*vcgi(1);
         -n(1)^2*rcgi(3)];
    
    % compute forces and torques on body i
    Force(:,i)=I(4,i)*fi;
    % already in inertial reference frame
end

% generalized forces for lagrange eq.
ext_f=gfa(a,J,Force,Torque,n_bodies);

% compute gravity forces acting on cg
Fcg=sum(Force,2);

if 0
    % generalized forces for lagrange eq.
ext_f=gfa(a,J,Force*0,Torque,n_bodies);

% compute gravity forces acting on cg
Fcg=[3*n(1)^2*xcg(1)+2*n(1)*sys.vcg(2);
                        -2*n(1)*sys.vcg(1);
         -n(1)^2*xcg(3)];

end



end




