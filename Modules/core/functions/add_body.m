function sys=add_body(sys,index,body)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

sys.bodies(index)=body;
sys.masses(index)=body.mass;
sys.Mtot=sum(sys.masses);

end