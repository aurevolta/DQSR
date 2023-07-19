function sys=add_gravity(sys)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

[A]=deal(zeros(8,sys.n_bodies,'like',sys.bodies(1).a));
[JJ]=deal(zeros(8,sys.nvar,sys.n_bodies,'like',sys.bodies(1).a));
II=zeros(4,sys.n_bodies);

for i=1:sys.n_bodies
    A(:,i)=sys.bodies(i).a;
    II(:,i)=[sys.bodies(i).inertia;sys.bodies(i).mass];
    JJ(:,:,i)=sys.bodies(i).J(:,:);
end

% compute gravity for all bodies
[gravity_f,sys.Fcg(:,1)]=sys.gravity_fun(sys,A,JJ,sys.xcg,II,sys.n_bodies);

% add forces on the bodies (these are inertial)
for i=1:sys.n_bodies
    sys.bodies(i).proj_f(:,2)=sys.bodies(i).proj_f(:,2)+gravity_f(:,i);
end

end