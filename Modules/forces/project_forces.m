function [Fcg,gen_forces]=project_forces(a,J,ext_f,Fcg0,body_forces,inner_forces,n_bodies)

% ext_f are forces already projected on the system whose projection on cg
% is Fcg
% body_forces are torques and forces [6xn_bodies] applied to each body
% inner_forces are internal actions directly applied to variables (ex controltorques on a joint)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

gen_forces=sum(gfa(a,J,body_forces(1:3,:),body_forces(4:6,:),n_bodies),2)+sum(ext_f,2)+sum(inner_forces,2);

Fcg=sum(body_forces(1:3,:),2)+Fcg0;


end