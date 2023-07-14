function [M,f,Fcg]=SYSlagrange(sys)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

M=zeros(sys.nvar);
f=zeros(sys.nvar,1);
Fcgi=zeros(3,sys.n_bodies);

for i=1:sys.n_bodies
    % compute for single body
    [Mi,fi,Fcgi(:,i)]=lagrange(sys.bodies(i),sys.dx);
    % sum up
    M=M+Mi;
    f=f+fi;
end

if sys.enforce_cg
    Fcg=sum([Fcgi,sys.Fcg],2);
else
    % for grounded systems
    Fcg=[0;0;0];
end

end