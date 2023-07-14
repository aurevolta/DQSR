function [A,DA,JJ,dJJ]=getfullstate(sys,n_bodies)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

[A,DA]=deal(zeros(8,n_bodies,'like',sys.bodies(1).a));

for i=1:size(A,2)
    DA(:,i)=dqprodm(sys.bodies(i).da)*[zq;sys.xcg/2;0]+dqprodm(sys.bodies(i).a)*[0;0;0;0;sys.vcg/2;0];
    A(:,i)=dqprodm(sys.bodies(i).a)*[zq;sys.xcg/2;0];
end



if nargout >2
    % if requested output also jacobians
    [~,n]=size(sys.bodies(1).J);
    [JJ,dJJ]=deal(zeros(8,n,sys.n_bodies,'like',sys.bodies(1).a));
    
    
    for i=1:size(A,2)
        JJ(:,:,i)=dqprodp([zq;sys.xcg/2;0])*sys.bodies(i).J(:,:);
        dJJ(:,:,i)=dqprodp([zq;sys.xcg/2;0])*sys.bodies(i).dJ(:,:)+dqprodp([zeros(4,1);sys.vcg/2;0])*sys.bodies(i).J(:,:);
    end
    
end


end