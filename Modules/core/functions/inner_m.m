function [M,dM,DMD]=inner_m(II,a,da)
% equivalent to S'*III*SS, faster computation

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% extract components
q=a(1:4);t=a(5:8);
dq=da(1:4);dt=da(5:8);


% matrix forms
QP=crossqp(q);
QMt=crossqm(q)';
TMt=crossqmt(t)';

dQP=crossqp(dq);
dQMt=crossqm(dq)';
dTMt=crossqmt(dt)';

% JJ=diag([II(1:3);sum(II(1:3))]);
JJ=diag([II(1:3);.5*sum(II(1:3))]);
QPJJ=QP*JJ;


% initialize mass amtrix
M=zeros(8,'like',a);

M(1:4,1:4)=4*QPJJ*(QP');
M([1,10,19,28])=M([1,10,19,28])+4*(t')*t*II(4);

M(1:4,5:8)=4*II(4)*TMt*QMt;
M(5:8,1:4)=M(1:4,5:8)'; % symmetric

M([37,46,55,64])=4*II(4);


% initialize derivative of mass matrix 
dM=zeros(8,'like',a);

dM(1:4,1:4)=4*(QPJJ*dQP'+dQP*JJ*QP');
dM([1,10,19,28])=dM([1,10,19,28])+8*(t')*dt*II(4);

dM(1:4,5:8)=4*II(4)*(dTMt*QMt+TMt*dQMt);
dM(5:8,1:4)=dM(1:4,5:8)'; % symmetric

% initialize DMD
DMD=zeros(8);


end