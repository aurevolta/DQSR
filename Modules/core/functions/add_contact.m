function SYSTEMS = add_contact(SYSTEMS)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

L=length(SYSTEMS);

global CONTACTABLE


if L>1
    % if more bodies are present, then compute contacts
    
    for i=1:L-1
        % compute contact forces for all systems (only once)
        
        
        % compute contacts for the selected system to the following systems
        SYSI=SYSTEMS(i);
        
        for j=i+1:length(SYSTEMS)
            
            % select the system to check for bodies contact
            SYSJ=SYSTEMS(j);
            
            for k=1:SYSI.n_bodies
                % for each bodies in system I
                Bk=SYSI.bodies(k);
                
                if Bk.ISGHOST
                    % if the body shall not be considered for contacts,
                    % then continue to the next one
                    continue
                end
                
                for l=1:SYSJ.n_bodies
                    % compute collision of BK and all bodies in system J
                    Bl=SYSJ.bodies(l);
                    
                    if Bl.ISGHOST
                        % if the body shall not be considered for contacts,
                        % then continue to the next one
                        continue
                    end
                    
                    % select the correct index of reference velocities
                    index=find(ismember(CONTACTABLE,[i,k,j,l],'rows'));
                    
                    % compute the contact
                    [Bk,Bl]=body2bodycontact(Bk,SYSI.xcg,SYSI.vcg...
                        ,SYSI.Mtot,Bl,SYSJ.xcg,SYSJ.vcg,SYSJ.Mtot,index(1));
                    
                    % update body L
                    SYSJ.bodies(l)=Bl;
                    
                end
                
                % update body K
                SYSI.bodies(k)=Bk;
            end
            
            % update system J
            SYSTEMS(j)=SYSJ;
        end
        
        % update system I
        SYSTEMS(i)=SYSI;
    end
    
end

end