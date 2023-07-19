function bodies_plot(sys,n,xi,body_id,boh)


% SPDX-License-Identifier: Apache-2.0
% 2016 Aureliano Rivolta


%%

coder.extrinsic('plot3')
coder.extrinsic('figure')
coder.extrinsic('quiver3')
coder.extrinsic('text')
coder.extrinsic('sprintf')
coder.extrinsic('axis')

if nargin >= 2
    figure(n)
else
    figure
end
hold on

if nargin<3
    xi=0.05;
end

if nargin < 4
body_id=ones(1,sys.n_bodies);
end


x=zeros(3,sys.n_bodies);
R=zeros(3,3,sys.n_bodies);

for i=1:sys.n_bodies
    
    [x(:,i),R(:,:,i)]=getkin(sys.bodies(i).a);
    
end

% centroid
C=mean(x,2);
% distance
D=sqrt(sum((x-repmat(C,1,sys.n_bodies)).^2));
% size
S=xi*max(D);


% base

if body_id(1)

plot3(x(1,1),x(2,1),x(3,1),'o','linewidth',2)
quiver3(x(1,1),x(2,1),x(3,1),R(1,1,1),R(1,2,1),R(1,3,1),S,'r','linewidth',2)
quiver3(x(1,1),x(2,1),x(3,1),R(2,1,1),R(2,2,1),R(2,3,1),S,'g','linewidth',2)
quiver3(x(1,1),x(2,1),x(3,1),R(3,1,1),R(3,2,1),R(3,3,1),S,'b','linewidth',2)
G=text(x(1,1),x(2,1),x(3,1),'base');
% G.FontSize=15;

end

% bodies
for i=2:sys.n_bodies
    
    if i==sys.n_bodies
        S=4*S;
    end
    
    if body_id(i)
    plot3(x(1,i),x(2,i),x(3,i),'o')
    quiver3(x(1,i),x(2,i),x(3,i),R(1,1,i),R(1,2,i),R(1,3,i),S,'r')
    quiver3(x(1,i),x(2,i),x(3,i),R(2,1,i),R(2,2,i),R(2,3,i),S,'g')
    quiver3(x(1,i),x(2,i),x(3,i),R(3,1,i),R(3,2,i),R(3,3,i),S,'b')
    G=text(x(1,i),x(2,i),x(3,i),sprintf('%d',i));
%     G.FontSize=15;
    end
    
end

axis equal
axis(3*S*[-1 1 -1 1 -1 1]+[C(1)*[1 1],C(2)*[1 1],C(3)*[1 1]])
xlabel x
ylabel y
zlabel z
grid minor


if nargin ==5

    for j=1:sys.n_bodies
        add_shape(sys.bodies(i));
    end
    
end

end





function [x,R]=getkin(a)

% % position
% x=2*crossqm(a(1:4))'*a(5:8);
% x(4)=[];
% % dcm
% R=q2dcm(a(1:4));
% 

[x,R]=dq2cartesian(a);


end


function add_shape(B,x,R)

if nargin == 1
    [x,R]=getkin(B.a);
end

P=R'*B.shape.P+repmat(x,1,size(B.shape.P,2));

pa=patch('Faces',B.shape.faces,'Vertices',P');


end







