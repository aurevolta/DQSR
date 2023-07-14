function [Ml,Fl,Fcg]=assembleCM(x,dx,xcg,vcg,time,II0,ext_f,sys_gen_fun,gravity_fun)
% ASSEMBLECM function
% This function call the system generation function to obtain all bodies
% states and jacobians w.r.t. problem variables. Then it assemble the mass
% matrix and the forcing term considering also constraints.
% THIS version is used for simulink
%
% call: 
% [Ml,Fl,Fcg]=assembleCM(x,dx,xcg,vcg,time,II0,ext_f,sys_gen_fun,gravity_fun)
% 
%%% Input
% * *x* multibody states
% * *dx* multibody states derivatives
% * *xcg* center of mass position
% * *vcg* center of mass velocity
% * *time* simulation time
% * *II0* mass and inertia properties
% * *ext_f* external input (forces or other nature)
% * *sys_gen_fun* matlab function that generates the system
% * *gravity_fun* gravity model to apply the system
%%% Output
% * *Ml* assembled extended mas matrix (include constraints)
% * *Fl* assembled applied forces 
% * *Fcg* forces applied to the center of mass 

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%% System generation
% 
% generate the system with the user defined *sys_gen_fun*, then enforces
% constraint, apply the user defined gravity model and computes the
% lagrangian of the system generating the basic mass matrix *Mb*, the forcing
% terms *Fl* and the forces applied to the center of mass *Fcg*

% call the user defined function to generate the system
[SYS]=sys_gen_fun(x,dx,xcg,vcg,time,II0,ext_f,[],gravity_fun);

% enforce the center of mass constraints
SYS=enforce_cg_constr(SYS);

% add gravity effects
SYS=add_gravity(SYS);

% compute system lagrangian
[Mb,ext_f,Fcg]=SYSlagrange(SYS);

% assemble forcing vector
Fl=sum(ext_f,2);

%% System constraint addition
% assemble final mass matrix with lagrangian multipliers rows and columns.
% To do so it generates an augmented mass matrix *Ml* and adds rows to *Fl*

% compute the _number_ of variables 
n_var=length(x);

% compute the number of constraints (if none it will return [] )
m=size(SYS.C,1); 

if m>0
    % generate augmented mass matrix
    Ml=zeros(n_var+m,n_var+m,'like',SYS.bodies(1).a);
    Ml(1:n_var,1:n_var)=Mb; % add the previous mass matrix
    Ml((n_var+1):end,1:n_var)=SYS.C;
    Ml(1:n_var,(n_var+1):end)=SYS.C';
    
    % if active constraints exist   
    Fl=[Fl;-SYS.dC*dx];
else
    Ml=Mb;
end

end







