function S=Sm(a)
% derivative of angular velocity and linear velocity with respect to the
% dual quaternion velocity. Used in the lagrangian formulation.

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% initialize
S=zeros(8,'like',a);

% qmatrix
Qa=2*crossqm(a(1:4))';

S(1:4,1:4)=2*crossqp(a(1:4))';
S(5:8,1:4)=2*crossqmt(a(5:8));
S(5:8,5:8)=Qa;

end