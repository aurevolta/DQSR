function sys=systemS(n_bodies,x,dx,xcg,vcg,time,gravity_fun,C,dC)


% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% initialize number of bodies and masses
sys.n_bodies=n_bodies;
sys.Mtot=0;
sys.masses=zeros(1,n_bodies);

% bodies struct & info
nvar=size(x,1);
sys.nvar=nvar;
B0=rigidbodyS([zeros(3,1);1],zdq,zeros(8,1),zeros(8,nvar),zeros(8,nvar),nvar);
sys.bodies=repmat(B0,1,n_bodies);

% CG state
sys.xcg=xcg;
sys.vcg=vcg;
sys.time=time;

% lagrange variables
sys.x=x;
sys.dx=dx;

% system lagrange eq
sys.M=zeros(nvar+size(C,1));
sys.f=zeros(nvar+size(C,1),1);
sys.Fcg=zeros(3,1);

% constraints
sys.C=C;
sys.dC=dC;

% utilities
sys.gravity_fun=gravity_fun;
sys.gravity_parameters=[];

% do enforce cg constraint
sys.enforce_cg=1;

end