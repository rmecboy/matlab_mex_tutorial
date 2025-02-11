% 三种不同的sfunction介绍 https://blog.csdn.net/weixin_38342580/article/details/128138667

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
  case 0,% 初始化
    [sys,x0,str,ts]=mdlInitializeSizes();
    
    %%%%%%%%%%%%%%%
    % Derivatives %
    %%%%%%%%%%%%%%%
    %case 1, % 连续时间导数
    %   sys=mdlDerivatives(t,x,u);
  case 2, % 更新离散状态量
    sys=mdlUpdate(t,x,u);
    %%%%%%%%%%%
    % Outputs %
    %%%%%%%%%%%
  case 3, % 计算输出
    sys=mdlOutputs(t,x,u);
    %case 4, % 计算下一步采样时刻
    %case 9, % 结束仿真
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
sizes.NumContStates = 0; %连续状态个数
sizes.NumDiscStates = 8; %离散状态个数
sizes.NumOutputs = 8; %输出量个数
sizes.NumInputs = 8; %输入量个数
sizes.DirFeedthrough = 1; %直接馈通标志
sizes.NumSampleTimes = 1; % 至少有一个采样时刻
sys = simsizes(sizes);
x0  = zeros(1,1);
str = [];
ts  = [0.000005 0];
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
else
  [sys(1),sys(2),sys(3),sys(4),sys(5),sys(6),sys(7),sys(8)] = simulink_sfunction(u(1),u(2),u(3),u(4),u(5),u(6),u(7),u(8));
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
