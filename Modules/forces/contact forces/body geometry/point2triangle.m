function [D,X]=point2triangle(P,Tp)
% minimum distance point to 3D triangle

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%% define the plane of the triangle
[n,D]=normaldir(Tp);

%% project all points on the plane

% point to triangle plane distance
point2plane=(n'*P)-D;

% project all points on triangle plane
Pp=-n*point2plane+P;

%% compute point to point distances

P2P=Tp-repmat(Pp,1,3);
P2Pd=sqrt(sum(P2P.^2)); % ditance

%% compute point to segment projection

% first (1-2)
n12=Tp(:,2)-Tp(:,1); % delta between 2 and 1, vector
d12=norm(n12); % norm of the vector
n12=n12/d12; % unit vector of direction
p12=-n12'*P2P(:,1); % projection of point to point 1 of line along the line
ggg=P2Pd(1)^2-p12^2;
if ggg>0
    dp12=sqrt(ggg); % distance between point and segment line
else
    dp12=0;
end

% second (2 -3)
n23=Tp(:,3)-Tp(:,2);
d23=norm(n23);
n23=n23/d23;
p23=-n23'*P2P(:,2);
ggg=P2Pd(2)^2-p23^2;
if ggg>0
    dp23=sqrt(ggg); % distance between point and segment line
else
    dp23=0;
end


% third (3 -1)
n31=Tp(:,1)-Tp(:,3);
d31=norm(n31);
n31=n31/d31;
p31=-n31'*P2P(:,3);
ggg=P2Pd(3)^2-p31^2;
if ggg>0
    dp31=sqrt(ggg); % distance between point and segment line
else
    dp31=0;
end



nn=[n12,n23,n31];
pp=[p12,p23,p31];
dd=[d12,d23,d31];
ddp=[dp12,dp23,dp31];

% check if the projection is inside the segment extrema
insideS=false(1,3);
for i=1:3
    if pp(i)>0 && pp(i)<dd(i)
        insideS(i)=true;
    end
end


%% determine distance

% check if the point projects inside the triangle
if sum(insideS)==3
    D=abs(point2plane); % the sign depends on the normal direction.
    X=Pp; % application point
    return
end

% if point is not inside triangle,then proceed to find the closest point
% and distance in the plane
% insideS
[Dp,index]=min([P2Pd,ddp(insideS)]);

if index<=3
    D=sqrt(Dp.^2+point2plane.^2);
    X=Pp+P2P(:,index);
else
    % determine the segment
    index=index-3;
    f=find(insideS);f=f(index);
    D=sqrt(ddp(f).^2+point2plane^2);
    X=Pp+P2P(:,f)+nn(:,f)*pp(f);
end

end