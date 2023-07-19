function [a,da,J,dJ]=enforce_cg(a0,da0,J0,dJ0,m)
% this function enforce the system to be centered in the center of mass and
% modifies jacobians accordingly. 

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% number of degrees of freedom
[~,dof,n]=size(J0);

% compute total mass
M=sum(m);
invM=1/M;

%compute mass fraction indeces
K=(1-invM*m);

% compute xi (aka the position*2 in the previous frame)

% initialize
[xi,dxi]=deal(zeros(4,n,'like',a0));
[Jxi,dJxi]=deal(zeros(4,dof,n,'like',a0));

for i=1:n
    % weighted relative positions
    xi(:,i)=m(i)*crossqm(a0(1:4,i))'*a0(5:8,i);
    dxi(:,i)=m(i)*(crossqm(a0(1:4,i))'*da0(5:8,i)+crossqm(da0(1:4,i))'*a0(5:8,i));
    
    %jacobians
    Jxi(:,:,i)=m(i)*(crossqm(a0(1:4,i))'*J0(5:8,:,i)+crossqmt(a0(5:8,i))*J0(1:4,:,i));
    dJxi(:,:,i)=m(i)*(crossqm(a0(1:4,i))'*dJ0(5:8,:,i)+crossqmt(a0(5:8,i))*dJ0(1:4,:,i)+...
        crossqm(da0(1:4,i))'*J0(5:8,:,i)+crossqmt(da0(5:8,i))*J0(1:4,:,i));
end

% update step
q=a0(1:4,:);
dq=da0(1:4,:);

% initialize
[a,da] = deal(zeros(size(a0),'like',a0));
[t,dt]=deal(zeros(4,n,'like',a0));
[J,dJ]=deal(zeros(8,dof,n,'like',a0));
J(1:4,:,:)=J0(1:4,:,:);
dJ(1:4,:,:)=dJ0(1:4,:,:);

% compute the new position quaternion
for i=1:n
    
    index=[1:(i-1),(i+1:n)];
    
    % sum of other contributions
    eta=sum(xi(:,index),2);
    deta=sum(dxi(:,index),2);    
    
    % jacobians
    
% THIS ONLY WHEN CHECKING WITH symbolic    
%     for u=1:4 
%         for j=1:dof
%             KKK(1,:)=Jxi(u,j,index);
%            Jeta(u,j)= sum(KKK);
%            KKK(1,:)=dJxi(u,j,index);
%             dJeta(u,j)= sum(KKK);
%         end
%     end
    
    Jeta=sum(Jxi(:,:,index),3);
    dJeta=sum(dJxi(:,:,index),3);    

    % position quaternion
    t(:,i)=K(i)*a0(5:8,i)-invM*crossqm(q(:,i))*eta;
    dt(:,i)=K(i)*da0(5:8,i)-invM*(crossqm(q(:,i))*deta+crossqm(dq(:,i))*eta);
    
    % jacobians
    J(5:8,:,i)=K(i)*J0(5:8,:,i)-invM*(crossqm(q(:,i))*Jeta+crossqp(eta)*J0(1:4,:,i));
    dJ(5:8,:,i)=K(i)*dJ0(5:8,:,i)-invM*(crossqm(q(:,i))*dJeta+crossqp(eta)*dJ0(1:4,:,i)+...
        crossqm(dq(:,i))*Jeta+crossqp(deta)*J0(1:4,:,i));
    
    % update state
    a(:,i) = [q(:,i);t(:,i)];
    da(:,i) = [dq(:,i);dt(:,i)];
end

% output the state
% a(:,:) = [q;t];
% da(:,:) = [dq;dt];

end