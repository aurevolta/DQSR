function [M,f,Fcg]=lagrange(body,dx)
% generate the mass/force contribution of this body

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% matrices
DS=Dm(body.da);
J=body.J(:,:);
dJ=body.dJ(:,:);

% determine mass matrix passing minimum number of parameters
III=gen_mass_princ(body.mass,body.inertia);

% compute inner mass amtrices and derivative with less computational
% burden
[M0,dM0]=inner_m([body.inertia;body.mass],body.a,body.da);

% mass matrix
M=J'*M0*J;

% forces
f=(-J'*M0*dJ*dx-J'*dM0*body.da+J'*DS'*III*DS*body.a); 

% add body,inertial and inner forces
body=body_forces(body);
body=inertial_forces(body);
body=inner_forces(body);


% update
f=f+sum(body.proj_f,2);

% forces projected on cg
Fcg=sum(body.Fcg,2);
end