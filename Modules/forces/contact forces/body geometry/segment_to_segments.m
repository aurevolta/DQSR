function [D,X]=segment_to_segments(S,indexS,D1,D2)
% iterate segment to segment distance for all instances of S indicated by
% indexS

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%%

% size of loop
ns=size(indexS,2);

% initialize
D=zeros(1,ns);
X=zeros(3,ns);

% compute
for i=1:ns
    [D(i),X(:,i)]=segment_to_segment(S(:,indexS(1,i)),S(:,indexS(2,i)),D1,D2);
end


end