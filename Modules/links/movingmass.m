function [rj,drj,Jr,dJr,x]=movingmass(x0,xf,x,dx)

% moving mass with no inertia. x0<x<xf

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

rj_axis=xf-x0;
L=norm(rj_axis);
rj_axis=rj_axis/L;

% check saturation
if x<0
    x=0;
end
if x>L
    x=L;
end

[rj,drj,Jr,dJr]=translational_joint(rj_axis,x,dx,0);

end