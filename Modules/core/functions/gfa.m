function [gfc]=gfa(a,J,forces,torques,n_bodies)
% this function computes the virtual displacement linked with generalized 
% forces and multiplies by forces. Forces and torques are in INERTIAL frame

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% generate 8d generalized forces
FF=[torques;zeros(1,n_bodies);forces;zeros(1,n_bodies)];

% initialize
gfc=zeros(length(J(1,:,1)),n_bodies);

% loop forces and toques contribution in the lagrangian formulation
for i=1:n_bodies
    % displacement w.r.t. dual quat. inertial forces
    G=[crossqm(a(1:4,i)),crossqpt(a(5:8,i))
        zeros(4),crossqm(a(1:4,i))];    
    % project onto free coordinates
    gfc(:,i)=2*J(:,:,i)'*G*FF(:,i);
end

end