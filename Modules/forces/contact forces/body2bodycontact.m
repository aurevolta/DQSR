function [B1c,B2c]=body2bodycontact(B1,XCG1,VCG1,m1,B2,XCG2,VCG2,m2,index)
% this function computes if there is a contact between body B1 and body B2,
% if so compute forces.

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%% duplicate bodies for output
B1c=B1;
B2c=B2;

%% determien relative positions and velocities
X1=dq2cartesian(dqprodp([zq;XCG1/2;0])*B1.a);
X2=dq2cartesian(dqprodp([zq;XCG2/2;0])*B2.a);

DX=norm(X2-X1);

%% check if bodies are near

if DX>=(B1.shape.R+B2.shape.R)
    % no collision possible, bodies too far
    return
end

%% get the points in a common reference frame

% get dcm
R1=q2dcm(B1.a(1:4));
R2=q2dcm(B2.a(1:4));

% get 3D points
P1=repmat(X1,1,size(B1.shape.P,2))+R1'*(B1.shape.P);
P2=repmat(X2,1,size(B2.shape.P,2))+R2'*(B2.shape.P);


%% compute the minimum distance and contact point

[d,X]=minimum_body_distance(P1,B1.shape.faces,X1,P2,B2.shape.faces,X2);
% X
% check if there is collision (millimiter tol)
% if (d)>5e-2
% if (d)>2e-3
%     % no collision possible, bodies too far
%     return
% end


%% if bodies are near get relative velocities

% get impact point for both bodies [using dqprodp since is inertial]
a1=dqprodp([zq;X/2;0])*B1.a;
a2=dqprodp([zq;X/2;0])*B2.a;
da1=dqprodp([zq;X/2;0])*(dqprodp([zq;XCG1/2;0])*B1.da+dqprodp([zeros(4,1);VCG1/2;0])*B1.a);
da2=dqprodp([zq;X/2;0])*(dqprodp([zq;XCG2/2;0])*B2.da+dqprodp([zeros(4,1);VCG2/2;0])*B2.a);

% % % % get relative dq (from 1) a2=dqprodm(r)*a1
% % % r=dqdivp(a1)*a2;
% % % dr=dqdivp(da1)*a2+dqdivp(a1)*da2;


% get impact points velocities
[x1,v1]=dq2cartesian(a1,da1);
[x2,v2]=dq2cartesian(a2,da2);




% % % % get the relative angle between the two velocities
% % % if norm(v1)== 0 || norm(v2) == 0
% % %     rel_ang=0;
% % % else
% % %     rel_ang=acosd(dot(v1/norm(v1),v2/norm(v2)));
% % % end
% % %
% % %
% % % if rel_ang>90
% % %     % bodies are getting farther from each other, no forces
% % %     return
% % % end

% get relative velocity and distance (already computed though, add a check)
vr=v2-v1;
xr=x2-x1;


% [norm(xr),d]

% % % get relative dq (from 1) a2=dqprodp(r)*a1 in inertial frame
% % r=dqdivm(a1)*a2;
% % dr=dqdivm(da1)*a2+dqdivm(a1)*da2;
% %
% %
% % % get impact point velocity
% % [xr,vr]=dq2cartesian(r,dr);

%% determine impact force

global coll_v_ref

% coll_v_ref=coll_v_ref+1;
% norm(v1-coll_v_ref(1,1:3)')
% if (d)>1e-3
%     % no collision possible, bodies too far
%     %     coll_v_ref(1,1:6)=[0,0,0,0,0,0];
%     if norm(v1-coll_v_ref(1,1:3)')<1e-3
%         coll_v_ref(1,1:6)=[0,0,0,0,0,0];
%         return
%     end
%     
%     
% elseif sum(coll_v_ref==zeros(size(coll_v_ref)))==length(coll_v_ref);
%     [v1r,v2r]=compute_reference_velocities(v1,v2,m1,m2,1);
%     coll_v_ref(1,1:6)=[v1r',v2r'];
%     111
% end
% coll_v_ref

d_tresh=1e-3;
is_v_ref_0=sum(coll_v_ref(:,index)==zeros(size(coll_v_ref(:,index))))==length(coll_v_ref(:,index));

if d>d_tresh
    % bodies are too far (they are farthening or collision control is still acting)
    if is_v_ref_0
        % then collision has been satisfied, no need to correct
        return
    end
    
else
    % bodies are close, if no reference is set, then set it, otherwise
    % conclude
    if is_v_ref_0
        [v1r,v2r]=compute_reference_velocities(v1,v2,m1,m2,0.5);
        coll_v_ref(1:6,index)=[v1r;v2r];
    end
    
end



% impact coefficients
Kp=1e6;

% inertial forces
% F1=slowcontact(d,vr); % reaction force
% F2=-F1; %opposite for the other body

v1r=coll_v_ref(1:3,index);
v2r=coll_v_ref(4:6,index);

F1=-Kp*(v1-v1r);
F2=-Kp*(v2-v2r);

% if target velocities are reached, then null the required velocities
if norm(v1-coll_v_ref(1:3,index))<1e-5
        coll_v_ref(1:6,index)=[0,0,0,0,0,0]';
end



% DF=vr;
%
% F1=F1+DF;
% F2=F2+DF;

% transport forces to bodies cg adding torques
% M1=-cross(X1-X,F1*1e-5*(m1*m2/(m1+m2)));
% M2=-cross(X2-X,F2*1e-5*(m1*m2/(m1+m2)));
M1=cross(X-X1,F1);
M2=cross(X-X2,F2);

% adjourne forces and torques
B1c.inertial_f(1:3)=B1c.inertial_f(1:3)+F1;
B1c.inertial_f(4:6)=B1c.inertial_f(4:6)+M1;
B2c.inertial_f(1:3)=B2c.inertial_f(1:3)+F2;
B2c.inertial_f(4:6)=B2c.inertial_f(4:6)+M2;





% coll_v_ref=vr';

end


% function [F]=slowcontact(xr,vr)
% 
% nxr=norm(xr);
% 
% if nxr>1e-3
%     C=1e-3;
%     C=1e-1*nxr;
%     C=1e-6*nxr^-1;
% else
%     C=2;
% end
% 
% F=C*(1e5*vr)*1e-3;
% 
% F=vr*80*1e3;
% 
% 
% end





