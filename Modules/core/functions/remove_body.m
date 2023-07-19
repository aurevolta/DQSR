function sys=remove_body(sys,index)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

sys.masses(index)=0;
sys.Mtot=sum(sys.masses);
sys.bodies(index)=[];
end