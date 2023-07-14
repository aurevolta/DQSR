function obj=inertial_forces(obj)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%% add contribution of inertial forces
[fin,obj.Fcg(:,2)]=gfa2(obj.a,obj.J,obj.inertial_f(1:3),obj.inertial_f(4:6),1);

obj.proj_f(:,2)=obj.proj_f(:,2)+fin;

end