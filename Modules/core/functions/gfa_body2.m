function [gfc,Fcg]=gfa_body2(a,J,forces,torques,x_appl,n_bodies)
% this function computes the virtual displacement linked with generalized forces and
% multiplies by forces. Forces and torques are in BODY frame

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% generate 8d generalized forces
FF=[torques;zeros(1,n_bodies);forces;zeros(1,n_bodies)];

% initialize
gfc=zeros(length(J(1,:,1)),1);
Fcg=zeros(3,1);

% loop forces and toques contribution in the lagrangian formulation
for i=1:n_bodies
    % select the application point of the force i
%     af=[a(1:4,i);crossqm([x_appl(:,i);0])*a(1:4,i)+a(5:8,i)];
    Q=crossqp(a(1:4,i)); % less computations
    af=[a(1:4,i);Q(1:4,1:3)*x_appl(:,i)+a(5:8,i)];
    H=dqprodm([zq;x_appl(:,i);0]);
    
    Rt=crossqp(af(1:4))*crossqm(af(1:4))';
    % displacement w.r.t dual quat, body forces
    G=[crossqp(af(1:4)),crossqpt(af(5:8))*Rt
        zeros(4),crossqp(af(1:4))];
    
    % project onto free coordinates
    gfc=gfc+2*J(:,:,i)'*H'*G*FF(:,i);
    
    % compute the projected forces on the cg
    Fcg=Fcg+Rt(1:3,1:3)*forces(:,i);
    
end

end

