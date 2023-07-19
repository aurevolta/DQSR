function [SYSTEMS]=assembleMCM(x,dx,xcg,vcg,time,II0,ext_f,sys_gen_fun,gravity_fun)

% this function call the system generation function to obtain all bodies
% states and jacobians w.r.t. problem variables. Then it assemble the mass
% matrix and the forcing term considering also constraints.
% THIS version is used for simulink

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta



%% systems generation

% number of variables (future: pass as argument)
% n_var=length(x);

% extract all the states
[SYSTEMS]=sys_gen_fun(x,dx,xcg,vcg,time,II0,ext_f,[],gravity_fun);


%% for each system

% initialize


for i=1:length(SYSTEMS)
    
    SYS=SYSTEMS(i);
    
    % enforce the constraints
    SYS=enforce_cg_constr(SYS);
    
    % add gravity effects
    SYS=add_gravity(SYS);
    
    % overwrite
    SYSTEMS(i)=SYS;
end

% add contact forces
SYSTEMS=add_contact(SYSTEMS);

for i=1:length(SYSTEMS)
    
    SYS=SYSTEMS(i);
    
    % compute system lagrangian
    [Mb,ext_f,Fcg]=SYSlagrange(SYS);
    
    % assemble final mass matrix with lagrangian multipliers
    
    % initialize
    m=size(SYS.C,1); % a.k.a. the number of constraints (if none it will return [] )
    Ml=SYS.M;
    Fl=SYS.f;
    
    
    % ssign the mass matrix
    Ml(1:SYS.nvar,1:SYS.nvar)=Mb;
    
    % add constraints
    if m>0
        Ml((SYS.nvar+1):(SYS.nvar+m),1:SYS.nvar)=SYS.C;
        Ml(1:SYS.nvar,(SYS.nvar+1):(SYS.nvar+m))=SYS.C';
    end
    
    % assemble forcing vector
    Fl(1:SYS.nvar,1)=sum(ext_f,2);
    
    % if active constraints exist
    if isempty(SYS.dC) == 0
        Fl((SYS.nvar+1):(SYS.nvar+m),1)=-SYS.dC*SYS.dx;
    end
    
    % store the matrices
    SYS.M=Ml;
    SYS.f=Fl;
    SYS.Fcg=Fcg;
    
    % overwrite
    SYSTEMS(i)=SYS;
    
end

end







