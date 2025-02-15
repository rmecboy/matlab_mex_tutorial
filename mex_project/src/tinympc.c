#include "tinympc.h"
#include <string.h>


// 生成高斯分布随机数 (均值0, 标准差sigma)
static float gaussian_noise(float sigma) {
    static float U1, U2;
    static int phase = 0;
    float Z;
    
    if (phase == 0) {
        U1 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
        U2 = rand() / (RAND_MAX + 1.0f);
        Z = sqrtf(-2.0f * logf(U1)) * cosf(2.0f * M_PI * U2);
    } else {
        Z = sqrtf(-2.0f * logf(U1)) * sinf(2.0f * M_PI * U2);
    }
    phase = 1 - phase;
    return Z * sigma;
}

// 生成噪声向量
static void add_process_noise(float noise[N_STATE], float sigma) {
    for (int i = 0; i < N_STATE; i++) {
        noise[i] = gaussian_noise(sigma);
    }
}
// 矩阵-向量乘法: y = M * x （M为rows x cols矩阵）
static void matrix_vector_mult(float y[], const float* M, const float x[], int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        y[i] = 0.0f;
        for (int j = 0; j < cols; j++) {
            y[i] += M[i * cols + j] * x[j];
        }
    }
}

// 矩阵-向量乘法: y = A * x
static void mat_vec_mult(float y[N_STATE], const float A[N_STATE][N_STATE], const float x[N_STATE]) {
    for (int i = 0; i < N_STATE; i++) {
        y[i] = 0.0f;
        for (int j = 0; j < N_STATE; j++) {
            y[i] += A[i][j] * x[j];
        }
    }
}

// 向量-标量乘加: y = y + a * x
static void vec_scalar_add(float y[N_STATE], const float x[N_STATE], float a) {
    for (int i = 0; i < N_STATE; i++) {
        y[i] += a * x[i];
    }
}

// 带约束参数的投影函数
static void project_constraints(float z[N_STATE], const float temp[N_STATE], 
                               const TinyMPC_Constraints* constraints) {
    memcpy(z, temp, sizeof(float)*N_STATE);
    // x2: 速度约束
    z[1] = fmaxf(fminf(temp[1], constraints->x2_max), -constraints->x2_max);
    // x5: 左摆杆角度约束
    z[4] = fmaxf(fminf(temp[4], constraints->x5_max), -constraints->x5_max);
    // x7: 右摆杆角度约束
    z[6] = fmaxf(fminf(temp[6], constraints->x7_max), -constraints->x7_max);
    // x9: 机体倾角约束
    z[8] = fmaxf(fminf(temp[8], constraints->x9_max), -constraints->x9_max);
}

// 初始化控制器
void tinympc_init(TinyMPC_Controller* ctrl, const TinyMPC_Model* model, float rho, const TinyMPC_Constraints* constraints) {
    ctrl->model = *model;
    ctrl->rho = rho;
    for (int i = 0; i < N_STATE; i++) {
        ctrl->x_ref[i] = 0.0f;
    }
     ctrl->constraints = *constraints;
        
}

// 设置参考轨迹
void tinympc_set_reference(TinyMPC_Controller* ctrl, const float x_ref[N_STATE]) {
    for (int i = 0; i < N_STATE; i++) {
        ctrl->x_ref[i] = x_ref[i];
    }
}

// 执行控制计算
// 执行控制计算
void tinympc_control(TinyMPC_Controller* ctrl, const float x[N_STATE], float u[N_INPUT]) {

    float X[N + 1][N_STATE] = {0}; // 预测时域状态序列
    float U[N][N_INPUT] = {0};     // 预测时域控制序列
    float p[N + 1][N_STATE] = {0}; // 梯度项
    float x_err[N_STATE];

    // 初始化当前状态
        memcpy(X[0], x, sizeof(float)*N_STATE);

    for (int i = 0; i < N_STATE; i++) {
        x_err[i] = x[i] - ctrl->x_ref[i];
    }

for (int iter = 0; iter < MAX_ADMM_ITER; iter++) 
    {
        // --- 后向传递计算p ---
        // 终端条件
        for (int i = 0; i < N_STATE; i++) {
            p[N][i] = ctrl->model.Q[i][i] * (X[N][i] - ctrl->x_ref[i])
                     + ctrl->rho * (ctrl->lambda[i] - ctrl->z[i]);
        }
        
        // 反向递归
        for (int k = N-1; k >= 0; k--) {
            float q_tilde[N_STATE];
            for (int i = 0; i < N_STATE; i++) {
                q_tilde[i] = ctrl->model.Q[i][i] * (X[k][i] - ctrl->x_ref[i])
                            + ctrl->rho * (ctrl->lambda[i] - ctrl->z[i]);
            }
            // p[k] = C2 * p[k+1] + q_tilde
            float temp[N_STATE];
            mat_vec_mult(temp, ctrl->model.C2, p[k+1]);
            for (int i = 0; i < N_STATE; i++) {
                p[k][i] = temp[i] + q_tilde[i];
            }
        }

      // --- 前向传递计算控制输入 ---
        for (int k = 0; k < N; k++) {
            // 计算控制输入u = -K*(X - x_ref) - C1*B'*p
            float Bp[N_INPUT] = {0};
            for (int i = 0; i < N_INPUT; i++) {
                for (int j = 0; j < N_STATE; j++) {
                    Bp[i] += ctrl->model.B[j][i] * p[k+1][j];
                }
            }
            for (int i = 0; i < N_INPUT; i++) {
                U[k][i] = 0.0f;
                for (int j = 0; j < N_STATE; j++) {
                    U[k][i] -= ctrl->model.K_inf[i][j] * (X[k][j] - ctrl->x_ref[j]);
                }
                for (int j = 0; j < N_INPUT; j++) {
                    U[k][i] -= ctrl->model.C1[i][j] * Bp[j];
                }
            }
            // 状态更新: X[k+1] = A*X[k] + B*U[k]
            float Ax[N_STATE], Bu[N_STATE];
            mat_vec_mult(Ax, ctrl->model.A, X[k]);
            matrix_vector_mult(Bu, (float*)ctrl->model.B, U[k], N_STATE, N_INPUT);
            for (int i = 0; i < N_STATE; i++) {
                X[k+1][i] = Ax[i] + Bu[i];
            }
        }
       // --- 松弛更新：时域投影 ---
    for (int k = 0; k < N; k++) {
        float temp[N_STATE];
        // 计算 temp = X[k] + lambda[k]/rho
        for (int i = 0; i < N_STATE; i++) {
            temp[i] = X[k][i] + ctrl->lambda[k][i] / ctrl->rho;
        }
        // 投影到约束集
        project_constraints(ctrl->z[k], temp, &ctrl->constraints);
    }

    // --- 对偶更新：lambda = lambda + rho*(X - z) ---
    for (int k = 0; k < N; k++) {
        for (int i = 0; i < N_STATE; i++) {
            ctrl->lambda[k][i] += ctrl->rho * (X[k][i] - ctrl->z[k][i]);
        }
    }
    // 应用第一个控制输入
    memcpy(u, U[0], sizeof(float)*N_INPUT);
}
}
void tinympc_state_update(float x[N_STATE], const float A[N_STATE][N_STATE], 
                          const float B[N_STATE][N_INPUT], const float u[N_INPUT], 
                          float noise_sigma) {
    float Ax[N_STATE], Bu[N_STATE], noise[N_STATE];
    
    // 计算 Ax = A * x
    matrix_vector_mult(Ax, (const float*)A, x, N_STATE, N_STATE);
    
    // 计算 Bu = B * u
    matrix_vector_mult(Bu, (const float*)B, u, N_STATE, N_INPUT);
    
    // 生成过程噪声
    add_process_noise(noise, noise_sigma);
    
    // 更新状态: x = Ax + Bu + noise
    for (int i = 0; i < N_STATE; i++) {
         x[i] = Ax[i] + Bu[i] + noise[i] ;
    }
}