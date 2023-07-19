function sys=enforce_cg_constr(sys)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

if sys.enforce_cg
    % otherwise do nothing
    
    [A,DA,JJ,dJJ]=getmatrices(sys);
    
    % enforce cg function
    [A,DA,JJ,dJJ]=enforce_cg(A,DA,JJ,dJJ,sys.masses);
    
    for i=1:sys.n_bodies
        sys.bodies(i).a=A(:,i);
        sys.bodies(i).da=DA(:,i);
        sys.bodies(i).J(:,:)=JJ(:,:,i);
        sys.bodies(i).dJ(:,:)=dJJ(:,:,i);
    end
    
end

end