function [v1r,v2r]=compute_reference_velocities(v10,v20,m1,m2,C)

% this function computes if there is a contact between body B1 and body B2,
% if so compute forces.

%%

M=m1+m2; % total mass
D=(1+C)/M; % if C = 0 anelastic, if C = 1 perfectly elastic

v1r=D*m2*(v20-v10)+v10;
v2r=D*m1*(v10-v20)+v20;



end