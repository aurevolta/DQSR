function [D,X]=segment_to_segment(X1,X2,X3,X4)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%% pre-compute
X12=X2-X1; % vector difference
D12=norm(X12); % length of segment
n12=X12/D12; % segment direction (from 1 to 2)

X34=X4-X3; % vector difference
D34=norm(X34); % length of segment
n34=X34/D34; % segment direction (from 1 to 2)

%% compute point to point distances

X13=X3-X1;
D13=norm(X13);
X14=X4-X1;
D14=norm(X14);

X23=X3-X2;
D23=norm(X23);
X24=X4-X2;
D24=norm(X24);


%% intersection point (special cases)

nc12=cross(n12,n34);
g=norm(nc12);

if g>0
    % unit vector of plane passing by 1 and 2, pointing normally at segment
    % 34
    nc12=nc12/g;
    
    % normal direction of plane passing by 3,4 and vector from intersect to
    % segment 34
    
    nc34=cross(nc12,n34);nc34=nc34/norm(nc34);
    % plane distance inverse
    Dc34=-mean(nc34'*[X3,X4]);
    
    % depth
    t=(-Dc34-nc34'*X1)/(n12'*nc34);
    
    %point of intersection
    P=X1+n12*t;
    
    if t<=D12 && t>=0
        % compute point to segment distance
        X3P=(X3-P); %difference
        pp_34=(-n34'*X3P); % projection
        
        if pp_34>0 && pp_34<D34
            Dp=norm(X3P+pp_34*n34); % distance
        else
            Dp=min([norm(X3P),norm(P-X4)]); % extrema
        end
        X=P;
        D=Dp;
        return
        
    else
        Dp=[];
        P=[];
    end
else
    
    % compute the normal plane
    
    [nn]=normaldir([X1,X2,X3]);
    
%     nn=pinv([X1,X2,X3,X4]')*-[1;1;1;1];
%     nn=nn/norm(nn);
    
    % get direction from 1 to 2
    z=cross(n12,nn);z=z/norm(z);
    
    % get the distance by using one fo the distances
    D=abs(z'*X14);
    
    % project point 3 and 4 to line [1,2]
    X3p=X3-z*D;
    X4p=X4-z*D;
    
    % project all points on the line
    XP=n12'*[X1,X2,X3p,X4p];
    
    % set X1 as reference
    XP=XP-XP(1);
    
    % get the intersect point
    PP=.5*(max([XP(1),min([max(XP(3:4)),XP(2)])])+min([XP(2),max([min(XP(3:4)),XP(1)])]));
    
    if min(abs(XP(3:4)))>=XP(2)
        % in case of outher segments add distances
        D=sqrt(D^2+(min(abs(XP(3:4)))-XP(2)).^2);
    end
    
    % get the 3D value
    X=X1+n12*PP;
    
    return
end


%% point to segment line distance

p3_12=n12'*X13;
p1_34=-n34'*X13;
p4_12=-n12'*X24;
p2_34=n34'*X24;

%% determine minimum distance point to segment

% 3 to [1,2]
if p3_12>=0 && p3_12<=D12
    V=-X13+n12*p3_12; %%
    D3_12=norm(V);
    X3_12=X3+V;
else
    [D3_12]=min([D13,D23]);
    X3_12=X3;
end

% 1 to [3,4]
if p1_34>=0 && p1_34<=D34
    V=X13-n34*p1_34;
    D1_34=norm(V);
    X1_34=X1;%+V; to project on other segment
else
    [D1_34]=min([D13,D14]);
    X1_34=X1;
end

% 4 to [1,2]
if p4_12>=0 && p4_12<=D12
    V=-X24-n12*p4_12;
    D4_12=norm(V);
    X4_12=X4+V;
else
    [D4_12]=min([D14,D24]);
    X4_12=X4;
end

%2 to [3,4]
if p2_34>=0 && p2_34<=D34
    V=X24-n34*p2_34;
    D2_34=norm(V);
    X2_34=X2;%+V; to project on other segment
else
    [D2_34]=min([D23,D24]);
    X2_34=X2;
end

%% final selection

D=[D1_34,D2_34,D3_12,D4_12,Dp];
X=[X1_34,X2_34,X3_12,X4_12,P];

[D,ind]=min(D);
X=X(:,ind);


end