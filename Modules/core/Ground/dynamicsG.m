function [ddx,a,da,dda]=dynamicsG(t,state,II,ext_f,sys_gen_fun,gravity_fun,n_bodies)
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
[Ml,Fl,a,da,J,dJ]=assembleG(x,dx,t,II,ext_f,sys_gen_fun,gravity_fun,n_bodies);

% compute the solution for attitude
sol=Ml\Fl;

% generate state derivative [acc;vel]
ddx=zeros(2*(n_var),1);
ddx(1:(n_var),1)=sol(1:n_var,1); % trow out the lagrangian multiplier
ddx((n_var+1):end,1)=dx;

% add accelerations
dda=zeros(8,n_bodies);
for i=1:n_bodies
   dda(:,i)=J(:,:,i)*ddx(1:(n_var),1)+dJ(:,:,i)*dx;
end


end