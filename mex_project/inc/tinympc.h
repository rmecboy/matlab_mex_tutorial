#ifndef TINYMPC_H
#define TINYMPC_H

#include <math.h>
#include <stdlib.h>
#include <math.h>

// 系统维度定义
#define N_STATE  10
#define N_INPUT   4
#define N_CONST   4  // 约束数量: x2, x5, x7, x9
#define MAX_ADMM_ITER 15
#define N 15                //预测时域

// 预计算矩阵结构体（需在MATLAB生成后填入）
typedef struct {
    float A[N_STATE][N_STATE];    // 系统矩阵
    float B[N_STATE][N_INPUT];    // 输入矩阵
    float K_inf[N_INPUT][N_STATE];// 稳态反馈增益
    float C1[N_INPUT][N_INPUT];   // (R + B^T P B)^{-1}
    float C2[N_STATE][N_STATE];   // (A - B*K_inf)^T
    float Q[N_STATE][N_STATE];    // 状态权重
    float R[N_INPUT][N_INPUT];    // 输入权重
} TinyMPC_Model;

// 状态约束结构体
typedef struct {
    float x2_max;  // 机体速度约束
    float x5_max;  // 左摆杆角度约束
    float x7_max;  // 右摆杆角度约束
    float x9_max;  // 机体倾角约束
} TinyMPC_Constraints;

// 控制器结构体
typedef struct {
    TinyMPC_Model model;          // 预计算模型参数
    float rho;                    // ADMM惩罚系数
    float x_ref[N_STATE];         // 参考状态
     float z[N + 1][N_STATE];      // 时域松弛变量
    float lambda[N + 1][N_STATE]; // 时域对偶变量
    TinyMPC_Constraints constraints; // 新增约束参数
} TinyMPC_Controller;

// 函数声明
void tinympc_init(TinyMPC_Controller* ctrl, const TinyMPC_Model* model, float rho, const TinyMPC_Constraints* constraints);
void tinympc_set_reference(TinyMPC_Controller* ctrl, const float x_ref[N_STATE]);
void tinympc_control(TinyMPC_Controller* ctrl, const float x[N_STATE], float u[N_INPUT]);
// 状态更新函数声明
void tinympc_state_update(float x[N_STATE], const float A[N_STATE][N_STATE], 
                          const float B[N_STATE][N_INPUT], const float u[N_INPUT], 
                          float noise_sigma);
#endif