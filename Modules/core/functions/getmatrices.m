function [A,DA,JJ,dJJ]=getmatrices(sys)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

[~,n]=size(sys.bodies(1).J);

[A,DA]=deal(zeros(8,sys.n_bodies,'like',sys.bodies(1).a));
[JJ,dJJ]=deal(zeros(8,n,sys.n_bodies,'like',sys.bodies(1).a));

for i=1:sys.n_bodies
    A(:,i)=sys.bodies(i).a;
    DA(:,i)=sys.bodies(i).da;
    JJ(:,:,i)=sys.bodies(i).J(:,:);
    dJJ(:,:,i)=sys.bodies(i).dJ(:,:);
end

end