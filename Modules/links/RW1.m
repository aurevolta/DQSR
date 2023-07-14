function [a,da,J,dJ]=RW1(Waxis,dqs0,Theta,dTheta,a0,da0,J0,dJ0,base_index,RW_index,RW_dof_index)
% Add a reaction wheel as body (RW_index) attached to body (base_index)
% occupying dofs (RW_dof_index). The axis is (Waxis) with (Theta & dTheta)
% as wheel states and with initial displacement of (dqs0). 

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta


%%

% determine dq and jacobians using the joint with no displacements
[rj,drj,Ja,Jb]=rotational_joint(Waxis,Theta,dTheta,zeros(3,1));


% repeated quantities
a=a0;
da=da0;
J=J0;
dJ=dJ0;
RJ=dqprodm(rj)*dqprodm(dqs0);
a00=a0(:,base_index);
da00=da0(:,base_index);
J00=J0(:,:,base_index);
RJP=dqprodp(a00)*dqprodp(dqs0);

% position and velocity
a(:,RW_index)=RJ*a00;
da(:,RW_index)=dqprodm(drj)*dqprodm(dqs0)*a00+RJ*da00;

% jacobians
[djj,jj]=deal(zeros(size(J00)));
jj(:,RW_dof_index)=Ja;
djj(:,RW_dof_index)=Jb;

J(:,:,RW_index)=RJ*J00+RJP*jj;
dJ(:,:,RW_index)=RJ*dJ0(:,:,base_index)+RJP*djj+...
    dqprodm(drj)*dqprodm(dqs0)*J00+dqprodp(da00)*dqprodp(dqs0)*jj;


end



