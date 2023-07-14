function [facez,dotp]=sort_faces(face,X,dir)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% initialize
nf=size(face,1);
n=zeros(3,nf);

for i=1:nf
    
    % take the points
    XP=X(:,face(i,:));
    
    % compute the normal
    [n(:,i)]=normaldir(XP);
    
end

% sort
[dotp,ind]=sort(abs(dir'*n),'ascend');

% output
facez=face(ind,:);

end

