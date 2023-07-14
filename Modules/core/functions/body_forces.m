function obj=body_forces(obj)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%% add contribution of body forces
[obj.proj_f(:,1),obj.Fcg(:,1)] = gfa_body2(obj.a,obj.J,obj.body_f(1:3),...
    obj.body_f(4:6),obj.body_f_loc,1);

end