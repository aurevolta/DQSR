function [D,X]=minimum_body_distance(P1,face10,X1,P2,face20,X2)

% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta

%% compute the intersection direction
DX=X2-X1;
n=DX/norm(DX);
D=0; % or D=norm(DX)/2;

deltP1=n'*P1-D;
% xi1=P1-n*deltP1;

delta2=n'*P2-D;
% xi2=P2-n*delta2;

% select closest points to one another
[~,ind1]=sort(deltP1,'descend');
[~,ind2]=sort(delta2,'ascend');

%% select faces not parallel to direction

if 1
[facez,dotp]=sort_faces(face10,P1,n);
face1=facez(dotp>cosd(89),:);

[facez,dotp]=sort_faces(face20,P2,n);
face2=facez(dotp>cosd(89),:);
end

% face index
indexS1=[face1(:,1:2)',face1(:,2:3)',face1(:,[3,1])'];
[~,b]=unique(prod(indexS1).*sum(indexS1));
indexS1=indexS1(:,b);

indexS2=[face2(:,1:2)',face2(:,2:3)',face2(:,[3,1])'];
[~,b]=unique(prod(indexS2).*sum(indexS2));
indexS2=indexS2(:,b);


%% segments to segments computation

% initialize
x5s=zeros(3,size(indexS1,2),size(indexS2,2));
D5s=zeros(size(indexS2,2),size(indexS1,2));

for j=1:size(indexS2,2)
    
    [D5s(j,:),x5s(:,:,j)]=segment_to_segments(P1,indexS1,P2(:,indexS2(1,j)),P2(:,indexS2(2,j)));
    
end

% find minimum values with 1e-6 tolerance
[i,j]=find(D5s<=(min(min(D5s))+1e-6));

% initialize
x5=zeros(3,length(i));
D5=zeros(1,length(i));

% search for minimum values
for k=1:length(i)
    x5(:,k)=x5s(:,j(k),i(k));
    D5(k)=D5s(i(k),j(k));
end

%% point to triangles

% initialize
[D1,D3]=deal(zeros(1,size(face1,1)));
[D2,D4]=deal(zeros(1,size(face2,1)));
[x1,x3]=deal(zeros(3,size(face1,1)));
[x2,x4]=deal(zeros(3,size(face2,1)));


% foremost 1 to 2
for i=1:size(face1,1)
    [D1(i),x1(:,i)]=point2triangle(P2(:,ind2(1)),P1(:,face1(i,:)));
end

% foremost 2 to 1
for i=1:size(face2,1)
    [D2(i),x2(:,i)]=point2triangle(P1(:,ind1(1)),P2(:,face2(i,:)));
end

% second foremost 1 to 2
for i=1:size(face1,1)
    [D3(i),x3(:,i)]=point2triangle(P2(:,ind2(2)),P1(:,face1(i,:)));
end

% second foremost 2 to 1
for i=1:size(face2,1)
    [D4(i),x4(:,i)]=point2triangle(P1(:,ind1(2)),P2(:,face2(i,:)));
end

%% final determination

% collect distances and impact points
DS=[D1,D2,D3,D4,D5];
xx=[x1,x2,x3,x4,x5];

% set indices to closest points with tolerance
indx=find(DS<=(min(DS)+1e-8));

% select the closest impact points and distance
PP=xx(:,indx);
D=DS(indx(1));

% eliminate duplicates
% PP=unique(round(PP',10+ceil(log10(max(max(abs(PP)))))),'rows','first')';

% determine centroid of impact
X=max_mean(PP);


end