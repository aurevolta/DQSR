function [ext_f,Fcg]=NO_gravity(sys,a,J,xcg,I,n_bodies)
% compute no gravity (single spherical main attractor)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% generalized forces for lagrange eq. 
ext_f=zeros(size(J,2),n_bodies);

% compute gravity forces acting on cg
Fcg=zeros(3,1);

end




