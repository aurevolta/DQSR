function DQ_viewer_vel(block)
%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the 
%   name of your S-function.
%
%   It should be noted that the MATLAB S-function is very similar
%   to Level-2 C-Mex S-functions. You should be able to get more
%   information for each of the block methods by referring to the
%   documentation for C-Mex S-functions.
%
%   Copyright 2003-2010 The MathWorks, Inc.

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

end

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 2;
block.NumOutputPorts = 0;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        = [8,1];
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

block.InputPort(2).Dimensions        = [8,1];
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;


% Register parameters
block.NumDialogPrms     = 2;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [block.DialogPrm(1).Data 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

block.RegBlockMethod('Outputs', @Outputs);     % Required


end


function Outputs(block)


a=block.InputPort(1).Data;
da=block.InputPort(2).Data;

% extract info
[X,V,OM,~,R,~]=dq2cartesian(a,da);

% closing the figure will pause simulation
if ishandle(block.DialogPrm(2).Data)==0
    pause
end

% open figure
figure(block.DialogPrm(2).Data)

%% attitude plot
subplot(1,2,1)
cla
hold on
color={'r','g','b'};%R=R';
for i=1:3
quiver3(0,0,0,R(i,1),R(i,2),R(i,3),1,'color',color{i})
end

% add inertial reference directions
text([1 0 0],[0 1 0],[0 0 1],{'x','y','z'})
plot3([0 1],[0 0],[0,0],'k--')
plot3([0 0],[0 1],[0,0],'k--')
plot3([0 0],[0 0],[0,1],'k--')


% add center direction and velocity direction
x=-X/norm(X);
v=V/norm(V);
om=OM/norm(OM);

quiver3(0,0,0,x(1),x(2),x(3),1,'color','k')
quiver3(0,0,0,v(1),v(2),v(3),1,'color','c')
quiver3(0,0,0,om(1),om(2),om(3),1,'color','m')

text(x(1),x(2),x(3),'0')
text(v(1),v(2),v(3),'V')





% grid on

% add sphere
[xs,ys,zs] = sphere(128);
h = surf(xs, ys, zs);
set(h, 'FaceAlpha', 0.1,'FaceColor',[1 0 0],'edgeColor','none')


th=linspace(0,2*pi,100);
c=[cos(th);sin(th);zeros(size(th))];

c1=R'*c;
c2=R'*angle2dcm(pi/2,0,0,'xyz')*c;
c3=R'*angle2dcm(pi/2,0,0,'yxz')*c;
plot3(c1(1,:),c1(2,:),c1(3,:),'k')
plot3(c2(1,:),c2(2,:),c2(3,:),'k')
plot3(c3(1,:),c3(2,:),c3(3,:),'k')



axis equal
axis([-1 1 -1 1 -1 1])
axis off



%% position plot
subplot(1,2,2)
hold on
plot3(X(1),X(2),X(3),'b.')
axis equal
% view(30,60)
xlabel 'x'
xlabel 'y'
xlabel 'z'

end


