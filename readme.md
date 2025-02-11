### 前置知识
- 关于Matlab/Simulink-S-function函数
    https://blog.csdn.net/weixin_38342580/article/details/128138667

- simulink中使用的sfuntion的类型为 Level-1 MATLAB S-Functions
    https://ww2.mathworks.cn/help/simulink/sfg/maintaining-level-1-matlab-s-functions.html#bq3i98j

### 编译方法
- 运行matalb_mex_make.m 即可编译 simulink_sfunction.mexw64

### 调用关系
    simulink仿真 
        -> sfuntion_c子模块 
            -> my_sfunction.m 
                -> simulink_sfunction.mexw64
    