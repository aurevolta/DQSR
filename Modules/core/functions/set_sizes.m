function obj=set_sizes(obj)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

obj.inner_f=zeros(obj.nvar,1);
obj.proj_f=zeros(obj.nvar,3);

obj.body_f=zeros(6,1);
obj.body_f_loc=zeros(3,1);

obj.inertial_f=zeros(6,1);

obj.inner_f=zeros(obj.nvar,1);

obj.Fcg=zeros(3,3); % projected forces on system cg (body,inertial,internal)

obj.proj_f=zeros(obj.nvar,3); % projected forces on dofs (body,inertial,internal


end