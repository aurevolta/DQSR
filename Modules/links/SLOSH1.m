function [a,da,J,dJ]=SLOSH1(SLaxis,SLpend,dqs0,Theta,dTheta,a0,da0,J0,dJ0,base_index,SL_index,SL_dof_index)
% Add a sloshing pendulum as body (SL_index) attached to body (base_index)
% occupying dofs (SL_dof_index). The axis is (Waxis) with (Theta & dTheta)
% as angular states and with initial displacement of (dqs0). 

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% determine dq and jacobians using the joint with no displacements
[rj1,drj1,Ja1,Jb1]=rotational_joint(SLaxis(:,1),Theta(1),dTheta(1),zeros(3,1));
% determine dq and jacobians using the joint with pendulum length
[rj2,drj2,Ja2,Jb2]=rotational_joint(SLaxis(:,2),Theta(2),dTheta(2),SLpend);

% repeated quantities
a=a0;
da=da0;
J=J0;
dJ=dJ0;
RJ=dqprodm(rj2)*dqprodm(rj1)*dqprodm(dqs0);

a00=a0(:,base_index);
da00=da0(:,base_index);
J00=J0(:,:,base_index);
RJP=dqprodp(a00)*dqprodp(dqs0);

% position and velocity
a(:,SL_index)=RJ*a00;
da(:,SL_index)=(dqprodm(drj2)*dqprodm(rj1)+dqprodm(rj2)*dqprodm(drj1))*dqprodm(dqs0)*a00+RJ*da00;

% jacobians 
[djj1,jj1,djj2,jj2]=deal(zeros(size(J00)));
jj1(:,SL_dof_index(1))=Ja1;
djj1(:,SL_dof_index(1))=Jb1;
jj2(:,SL_dof_index(2))=Ja2;
djj2(:,SL_dof_index(2))=Jb2;

J(:,:,SL_index)=RJ*J00+RJP*dqprodm(rj2)*jj1+RJP*dqprodp(rj1)*jj2;


dJ(:,:,SL_index)=RJ*dJ0(:,:,base_index)+...
    RJP*dqprodm(rj2)*djj1+RJP*dqprodp(rj1)*djj2+...
    (dqprodm(drj2)*dqprodm(rj1)+dqprodm(rj2)*dqprodm(drj1))*dqprodm(dqs0)*J00+...
    RJP*dqprodm(drj2)*jj1+RJP*dqprodp(drj1)*jj2;


end



