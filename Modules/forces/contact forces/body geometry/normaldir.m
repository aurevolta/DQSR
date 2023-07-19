function [n,D]=normaldir(XP)
% [n]=normaldir(XP)
% normal direction to plane passing by XP points

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta


%%

nt=XP'\ones(size(XP,2),1);

if sum(isinf(nt)) || sum(isnan(nt))
    % then the normal is on one of the principal axis
    
    nx=[1,0,0]';
    ny=[0,1,0]';
    nz=[0,0,1]';
    
    Dx=nx'*XP;
    Dy=ny'*XP;
    Dz=nz'*XP;
    
    if sum(Dx)==(Dx(1)*size(Dx,2))
        n=nx;
    elseif sum(Dy)==(Dy(1)*size(Dy,2))
        n=ny;
    elseif sum(Dz)==(Dz(1)*size(Dz,2))
        n=nz;
    else
        % collinear points?
%         n=NaN(3,1);
        n=cross(XP(:,1),XP(:,2));
    end
else
    n=nt;
end

D=1/norm(n);
n=n*D;

end