function II=gen_body_mass(m,J)
% This function generates the 8x8 mass matrix comprising both mass and
% inertia properties. Takes input the mass (scalar) and the 3x3 body
% relative inertia. 

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% m is the mass, J the 3x3 inertia

II=zeros(8);
II(1:3,1:3)=J;
II(4,4)=.5*trace(J);
II(5,5)=m;
II(6,6)=m;
II(7,7)=m;
II(8,8)=m;

end
