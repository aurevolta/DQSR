function [Y]=max_mean(X)
% find the center of sphere encapsulating all points X

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% number of elements
n=size(X,2);

% if just one or two elements, just send out the mean
if n==2
    Y=mean(X(:,1:2),2);
    return
elseif n==1
    Y=X;
    return
end


XP=X(:,1:2);
for j=3:n
    
    % compute the 3 mean
    Mj=mean([XP,X(:,j)],2);
    
    % find the internal point between the three 
    % (if in circle, it does not affect negatively)
    G=[XP,X(:,j)];
    DJ=sqrt(sum((G-repmat(Mj,1,3)).^2));
    [~,ind]=min(DJ);
    
    % adjourne
    H=G(:,[1,2,3]~=ind);
    XP=H(:,1:2);
    
end

% output the center of circle of innermost and outermost elements
Y=mean(XP,2);

end