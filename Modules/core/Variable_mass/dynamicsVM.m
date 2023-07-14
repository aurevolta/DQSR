function [ddx,dxi]=dynamicsVM(t,state,xcg,vcg,xi,II,ext_f,sys_gen_fun,gravity_fun)
% This function takes the assembled matrices and compute the state
% derivative. 

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%% computation 

% state dimension
n_var=length(state)/2;

% get state and its derivative
dx=state(1:n_var);
x=state(n_var+1:end);

% assemble the mass matrix and forcing term
[Ml,Fl,Fcg,dxi]=assembleVM(x,dx,xcg,vcg,xi,t,II,ext_f,sys_gen_fun,gravity_fun);

% compute the solution for attitude
sol=Ml\Fl;

% compute solution for position
M=sum(II(end,:)); % total mass
dvcg=Fcg/M;

% generate state derivative [acc;vel]
ddx=zeros(2*(n_var+3),1);
ddx(1:(n_var),1)=sol(1:n_var,1); % trow out the lagrangian multiplier

ddx(n_var+1:n_var+3)=dvcg;

ddx((n_var+4):end-3,1)=dx;
ddx((end-2):end,1)=vcg;





end