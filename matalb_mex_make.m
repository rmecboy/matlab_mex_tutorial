clc

delete('simulink_sfunction.mexw64');
delete('./log/mex_build_log.txt');

diary('./log/mex_build_log.txt');


NOW_PATH = '.';
SIMULINK_SFUNCTION_RTEPATH = './mex_project'
INC_PATH = './mex_project/inc'
SRC_PATH = './mex_project/src'

% 'CFLAGS=$CFLAGS -O0', 'COPTIMFLAGS=-O0', 'CXXFLAGS=-$CXXFLAGS -O0', 'CXXOPTIMFLAGS=-O0', 'LDFLAGS=$LDFLAGS -O0', ...
result = mex(['-DMATLAB_SIM'],...
    ['-I', NOW_PATH],['-I', SIMULINK_SFUNCTION_RTEPATH], ...
    ['-I', INC_PATH],['-I', SRC_PATH], ...
    fullfile(SIMULINK_SFUNCTION_RTEPATH, 'simulink_sfunction.c'),...
    fullfile(SRC_PATH, 'sim_main.c'),...
    ['-silent']);

diary off;
