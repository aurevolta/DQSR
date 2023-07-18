function [DIM,II2,endeff,end_eff_index,laserDQ,laserDQarm,cam_e_dq,cam_e_id,cam_l_dq,cam_r_dq,cam_lr_id,baseline,...
    Mc,Nc,Oc,Kc,Gc,Ic,KV,KP,TV,DT_wakeup,PP,dPP,PM,THR]=chaser_robot_data

% mass
m_sat2=1500;

% sides
x_sat=2;
y_sat=1;
z_sat=1;
DIM=[x_sat;y_sat;z_sat];

% inertia
I_sat2=m_sat2/12*diag([y_sat^2+z_sat^2,x_sat^2+z_sat^2,x_sat^2+y_sat^2]);

% masses
rho=2700; %kg/m3
r=0.05; % m
% lengths=[0.1,1,0.2,0.8,0.1,0.1,0.05];
[lengths,rj_axis,l_cg,b_cg]=OR_robotic_arm_parameters;


% lengths=lengths.*[1 0 1 0 1 0 1];

m=[m_sat2,pi*r.^2.*lengths*rho];


% links inertias
Izz=m(2:end).*r.^2./2;
Ixx=m(2:end)./12.*(3*r.^2+lengths.^2);


% collect together
II2(1:3,1)=diag(I_sat2);
II2(1:3,2:8)=((l_cg(:,2:end)+b_cg)~=0).*([1;1;1]*Ixx)+((l_cg(:,2:end)+b_cg)==0).*([1;1;1]*Izz);
II2(4,:)=m;


% end effector
q_ee=[0 1 0 1]';q_ee=q_ee/norm(q_ee);
% d_ee=[0;0;lengths(end)];
% d_ee=l_cg(:,end)+b_cg(:,end);
d_ee=-b_cg(:,end);
% d_ee=[.5;0;0]
endeff=[q_ee;crossqp([d_ee;0]/2)*q_ee];
end_eff_index=8;



%%  sensors

% laser positioning
% positive x direction, z axis for laser [0;0;1]
ql=[0 1 0 1]'*sqrt(2)/2;
tl=crossqm(ql)*[[x_sat/2;0;0]/2;0];
laserDQ=[ql;tl];
laserDQarm=dqprodm([0;0;0;1;[0;0;-.05];0])*dqprodm([1;zeros(7,1)])*endeff;


% cameras dq

% end effector
cam_e_dq=dqprodm([0 1 0 0,0 0 0 0]')*endeff;
cam_e_id=8; % end effector


% body mounted
q_0=[0 sqrt(2)/2 0 sqrt(2)/2]';

q_0r=[0 0 -sqrt(2)/2 sqrt(2)/2]';

baseline=.2;
x_0l=[0;baseline/2;x_sat/2+0.05*0;0];
x_0r=[0;-baseline/2;x_sat/2+0.05*0;0];
cam_l_dq=[q_0;crossqm(x_0l/2)*q_0];
cam_r_dq=[q_0;crossqm(x_0r/2)*q_0];

% rotate to set the camera in the right z rotation (x,y)
cam_l_dq=dqprodm([q_0r;zeros(4,1)])*cam_l_dq;
cam_r_dq=dqprodm([q_0r;zeros(4,1)])*cam_r_dq;

cam_lr_id=1;

DRCAM=dqdivp(cam_l_dq)*cam_r_dq;

baseline=[eye(3),zeros(3,1)]*2*crossqm(DRCAM(1:4))'*DRCAM(5:8);



%% chaser control parameters

% base attitude
Mc=diag([1 1 1 0])*2.5e-4;
Nc=diag([1 1 1 1])*15e-2;
Oc=diag([1 1 1 0])*1e-7;



% base position (TBC)
Cscale=.5*20*1e-3;
Kc=Cscale*diag([1 1 1 1])*1e-1;
Gc=Cscale*diag([1 1 1 1])*5e1;
Ic=1e-5*1e-2;%*1e-3;

% joints
KV=diag([0.2500
    0.1000
    0.0500
    0.1000
    0.0500
    0.0050
    0.0200]);

KP=diag([1.0500
    3.0000
    1.0000
    3.0000
    1.0000
    0.6000
    0.0200]);

TV =[30
    30
    30
    30
    30
    30
    30];

%% arm wakeup path

% DT_wakeup=500;
DT_wakeup=300;
% DT_wakeup=200;


% DTH=[pi/6*0,pi/3+pi/2,0,-pi/2-pi/3,0,-pi/2,0]';
DTH=[0,4*pi/6,0,-4*pi/6,0,-pi/2,0]';

[Theta,dTheta]=deal(zeros(1,7));
Thetaf=DTH';
dThetaf=zeros(7,1)';

PP=zeros(7,6);
dPP=zeros(7,5);
ddPP=zeros(7,4);

for i=1:7
    
    pp3=(20*(Thetaf(i)-Theta(i))-(8*dThetaf(i)+12*dTheta(i))*DT_wakeup)/(2*DT_wakeup^3);
    pp4=(-30*(Thetaf(i)-Theta(i))+(14*dThetaf(i)+16*dTheta(i))*DT_wakeup)/(2*DT_wakeup^4);
    pp5=(12*(Thetaf(i)-Theta(i))-6*(dThetaf(i)+dTheta(i))*DT_wakeup)/(2*DT_wakeup^5);
    
    
    PP(i,end:-1:1)=[Theta(i) dTheta(i) 0 pp3 pp4 pp5];
    
    dPP(i,:)=polyder(PP(i,:));
    ddPP(i,:)=polyder(dPP(i,:));
     
end



%% propulsors location 

pmf=[1 1 1 1 -1 -1 -1 -1];
pmt1=@(x) [x -x x -x x -x x -x];
% pmt2=@(x) [-x x -x x -x x -x x];
pmt2=@(x) [x x -x -x x x -x -x];
pmz=zeros(1,8);


PM=[pmz,pmt1(z_sat/2),pmt1(y_sat/2)
    pmt2(z_sat/2),pmz,pmt2(x_sat/2)
    pmt1(y_sat/2),pmt2(x_sat/2),pmz
    pmf,pmz,pmz
    pmz,pmf,pmz
   pmz,pmz,pmf];

THR=5;%.05; % thrust of each engine

PM=PM*THR; 



end