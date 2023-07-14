function [DDX,dcg]=dynamicsMCM(t,state,xcg,vcg,II,ext_f,sys_gen_fun,gravity_fun)
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
[SYSTEMS]=assembleMCM(x,dx,xcg,vcg,t,II,ext_f,sys_gen_fun,gravity_fun);

% initialize
dvcg=zeros(size(vcg));
ddx=zeros(size(dx));
index=1;

for i=1:length(SYSTEMS)
    % compute the solution for attitude
    sol=SYSTEMS(i).M\SYSTEMS(i).f;
    
    % add the solution
    ddx(index:(index+SYSTEMS(i).nvar-1))=sol(1:SYSTEMS(i).nvar,1); % trow out the lagrangian multiplier
    
    % update the index
    index=index+SYSTEMS(i).nvar;
    
    % compute solution for cg
    dvcg(:,i)=SYSTEMS(i).Fcg/SYSTEMS(i).Mtot;
end

% variables derivatives
DDX=[ddx;dx];

% centers of mass derivatives
dcg=[dvcg;vcg];

end