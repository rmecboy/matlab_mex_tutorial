% ���ֲ�ͬ��sfunction���� https://blog.csdn.net/weixin_38342580/article/details/128138667

% Level-1 MATLAB S-Function 
function [sys,x0,str,ts] = Outloop(t,x,u,flag)

%CSFUNC An example MATLAB file S-function for defining a continuous system.
%   Example MATLAB file S-function implementing continuous equations:
%      x' = Ax + Bu
%      y  = Cx + Du
%
%   See sfuntmpl.m for a general S-function template.
%
%   See also SFUNTMPL.

%   Copyright 1990-2009 The MathWorks, Inc.
switch flag,
  
  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,% ��ʼ��
    [sys,x0,str,ts]=mdlInitializeSizes();
    
    %%%%%%%%%%%%%%%
    % Derivatives %
    %%%%%%%%%%%%%%%
    %case 1, % ����ʱ�䵼��
    %   sys=mdlDerivatives(t,x,u);
  case 2, % ������ɢ״̬��
    sys=mdlUpdate(t,x,u);
    %%%%%%%%%%%
    % Outputs %
    %%%%%%%%%%%
  case 3, % �������
    sys=mdlOutputs(t,x,u);
    %case 4, % ������һ������ʱ��
    %case 9, % ��������
    %%%%%%%%%%%%%%%%%%%
    % Unhandled flags %
    %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];
    
    %%%%%%%%%%%%%%%%%%%%
    % Unexpected flags %
    %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    
end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%

function [sys,x0,str,ts]=mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates = 0; %����״̬����
sizes.NumDiscStates = 10; %��ɢ״̬����
sizes.NumOutputs = 10; %���������
sizes.NumInputs = 10; %����������
sizes.DirFeedthrough = 1; %ֱ����ͨ��־
sizes.NumSampleTimes = 1; % ������һ������ʱ��
sys = simsizes(sizes);
x0  = zeros(1,1);
str = [];
ts  = [0.002 0];
% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
% function sys=mdlDerivatives(t,x,u)
%
% sys = A*x + B*u;

function sys=mdlUpdate(t,x,u)
if t < 0.001
  sys(1) = 0;
  sys(2) = 0;
  sys(3) = 0;
  sys(4) = 0;
  sys(5) = 0;
  sys(6) = 0;
  sys(7) = 0;
  sys(8) = 0;
  sys(9) = 0;
  sys(10) = 0;
else
  [sys(1),sys(2),sys(3),sys(4),sys(5),sys(6),sys(7),sys(8),sys(9),sys(10)] = simulink_sfunction(u(1),u(2),u(3),u(4),u(5),u(6),u(7),u(8),u(9),u(10));
end
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
function sys = mdlOutputs(t,x,u)
sys(1) = x(1);
sys(2) = x(2);
sys(3) = x(3);
sys(4) = x(4);
sys(5) = x(5);
sys(6) = x(6);
sys(7) = x(7);
sys(8) = x(8);
sys(9) = x(9);
sys(10) = x(10);
