function [force,torque,dxi]=variable_mass_equivalent_forces(M,xi,dIdxi,omega,Thrust,vr)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta


%%

% propulsion force


VR=sqrt(sum(vr.^2));

% mass variation
dxi=-sum(Thrust./VR)/M;


if xi==0
    % empty tank
    [force,torque]=deal(zeros(3,1)); 
else

% forces given direction vr/VR
force=Thrust./VR.*vr;

% torque due to variable inertia
torque=-dxi*dIdxi*omega;
end


end