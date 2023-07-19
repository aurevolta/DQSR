function obj=inner_forces(obj)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

obj.proj_f(:,3)=obj.inner_f;
obj.Fcg(:,3)=zeros(3,1); % by definition
end