function [L_tot,LJ_tot] = Total_angular_momentum(sys)
% computes the total angular momentum of a system

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

n=sys.n_bodies;

L=zeros(3,n,'like',sys.bodies(1).a);
LJ_tot=zeros(3,sys.nvar,'like',sys.bodies(1).a);

for i=1:n
    II=[sys.bodies(i).inertia;sys.bodies(i).mass];
    [ L(:,i),LJ ] = angular_momentum(II,sys.bodies(i).a,sys.bodies(i).da,sys.bodies(i).J);
    LJ_tot=LJ_tot+LJ;
end

% total angular momentum
L_tot=sum(L,2);


end

