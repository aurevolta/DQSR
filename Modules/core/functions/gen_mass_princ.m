function II=gen_mass_princ(m,J)
% This function generates the 8x8 mass matrix comprising both mass and
% inertia properties. Takes input the mass (scalar) and the 3x3 body
% relative inertia. All in the principal axis frame of the body
% OLD

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% m is the mass, J the inertia vector [Ixx;Iyy;Izz]

II=diag([J;(.5*sum(J));m;m;m;m]);

end
