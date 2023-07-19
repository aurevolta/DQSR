function [Ml,Fl,a,da,J,dJ]=assembleG(x,dx,time,II0,ext_f,sys_gen_fun,gravity_fun,n_bodies)

% this function call the system generation function to obtain all bodies
% states and jacobians w.r.t. problem variables. Then it assemble the mass
% matrix and the forcing term considering also constraints.
% THIS version is used for simulink

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta



%% system generation

% number of variables (future: pass as argument)
n_var=length(x);

% extract all the states
[SYS]=sys_gen_fun(x,dx,[],[],time,II0,ext_f,[],gravity_fun);

% add gravity effects
SYS=add_gravity(SYS);

% compute system lagrangian
[Mb,ext_f]=SYSlagrange(SYS);

% assemble final mass matrix with lagrangian multipliers
m=size(SYS.C,1); % a.k.a. the number of constraints (if none it will return [] )
Ml=zeros(n_var+m,n_var+m,'like',SYS.bodies(1).a);
Ml(1:n_var,1:n_var)=Mb;

if m>0
    Ml((n_var+1):end,1:n_var)=SYS.C;
    Ml(1:n_var,(n_var+1):end)=SYS.C';
end

% assemble forcing vector
Fl=sum(ext_f,2);

% if active constraints exist
if isempty(SYS.dC) == 0
    Fl=[Fl;-SYS.dC*dx];
end

%% State update with cg
[a,da,J,dJ]=getmatrices(SYS);

end







