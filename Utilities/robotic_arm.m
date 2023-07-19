function [a,da,J,dJ]=robotic_arm(a0,da0,rj_axis,Theta,dTheta,jointK,d1,d2,dq_static0,dofbase,J0base)%#codegen

% Easily computes all the joints for a robotic arm. 

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta


%%

[~,l_chain]=size(rj_axis);
l_chain=l_chain+1;

% initialize
[rj,drj]=deal(zeros(8,l_chain,'like',Theta));
[Ji,dJi]=deal(zeros(8,l_chain,'like',Theta));

% ADD DOF DEPENDENCE ON SIZE OF Ji and following (for mexing)
coder.varsize('Ji',[8 8+l_chain]);
coder.varsize('dJi',[8 8+l_chain]);

dof=dofbase+(l_chain)-1;

for i=2:l_chain
    switch jointK(i-1)
        case 'R'
            [rj(:,i),drj(:,i),Ja,Jbb]=rotational_joint2(rj_axis(:,i-1),Theta(i-1),dTheta(i-1),d1(:,i-1),d2(:,i-1));
        case 'T'
            [rj(:,i),drj(:,i),Ja,Jbb]=translational_joint(rj_axis(:,i-1),Theta(i-1),dTheta(i-1),0);
        otherwise
            rj=zdq;
            drj=zeros(8,1);
            Ja=zeros(8,1);
            Jbb=zeros(8,1);
    end
    
    % store jacobians
    Ji(:,i)=Ja;
    dJi(:,i)=Jbb;
    
end

% initialize
a=zeros(8,l_chain,'like',Theta);
da=zeros(8,l_chain,'like',Theta);
[J,dJ]=deal(zeros([8,dof,l_chain],'like',Theta));

% base
J(1:8,1:dofbase,1)=J0base;
a(:,1)=a0;
da(:,1)=da0;

% first joint
D0A0=dqprodm(dq_static0)*a0;
D0DA0=dqprodm(dq_static0)*da0;

a(:,2)=dqprodm(rj(:,2))*D0A0;
da(:,2)=dqprodm(rj(:,2))*D0DA0+dqprodm(drj(:,2))*D0A0;

J(:,:,2)=dqprodm(rj(:,2))*dqprodm(dq_static0)*J(:,:,1);
J(:,dofbase+1,2)=dqprodp(D0A0)*Ji(:,2);

dJ(:,:,2)=dqprodm(rj(:,2))*dqprodm(dq_static0)*dJ(:,:,1)+dqprodm(drj(:,2))*dqprodm(dq_static0)*J(:,:,1);
dJ(:,dofbase+1,2)=dqprodp(D0A0)*dJi(:,2)+dqprodp(D0DA0)*Ji(:,2);


% second joint onwards
for i=3:l_chain
    
    a(:,i)=dqprodm(rj(:,i))*a(:,i-1);
    da(:,i)=dqprodm(rj(:,i))*da(:,i-1)+dqprodm(drj(:,i))*a(:,i-1);
    
    J(:,:,i)=dqprodm(rj(:,i))*J(:,:,i-1);
    J(:,dofbase-1+i,i)=dqprodp(a(:,i-1))*Ji(:,i);
    
    dJ(:,:,i)=dqprodm(rj(:,i))*dJ(:,:,i-1)+dqprodm(drj(:,i))*J(:,:,i-1);
    dJ(:,dofbase-1+i,i)=dqprodp(a(:,i-1))*dJi(:,i)+dqprodp(da(:,i-1))*Ji(:,i);
end

end





