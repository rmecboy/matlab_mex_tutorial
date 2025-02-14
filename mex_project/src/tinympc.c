#include "tinympc.h"


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

// 向量投影到约束
static void project_constraints(float z[N_STATE], const float temp[N_STATE], float rho) {
    const int constrained_indices[N_CONST] = {1,4,6,8}; // x2,x5,x7,x9
    const float limits[N_CONST] = {3.0f,0.2f,0.2f,0.2f};
    
    for (int i = 0; i < N_CONST; i++) {
        int idx = constrained_indices[i];
        float temp_val = temp[idx];
        z[idx] = fmaxf(fminf(temp_val, limits[i]), -limits[i]);
    }
    // 其他状态保持原值
    for(int i=0; i<N_STATE; i++){
        if(i != 1 && i !=4 && i !=6 && i !=8) 
            z[i] = temp[i];
    }
}

// 初始化控制器
void tinympc_init(TinyMPC_Controller* ctrl, const TinyMPC_Model* model, float rho) {
    ctrl->model = *model;
    ctrl->rho = rho;
    for (int i = 0; i < N_STATE; i++) {
        ctrl->x_ref[i] = 0.0f;
        ctrl->z[i] = 0.0f;
        ctrl->lambda[i] = 0.0f;
    }
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
    float x_err[N_STATE];
    for (int i = 0; i < N_STATE; i++) {
        x_err[i] = x[i] - ctrl->x_ref[i];
    }

    for (int iter = 0; iter < MAX_ADMM_ITER; iter++) {
        // 后向传递，维护每个时间步的p
        float p[N + 1][N_STATE] = {0}; // p[0]到p[N]
        
        // 计算终端的q_tilde（此处假设q_tilde在终端时间步为0）
        for (int i = 0; i < N_STATE; i++) {
            p[N][i] = ctrl->rho * (ctrl->lambda[i] - ctrl->z[i]);
        }

        // 从时域末端反向迭代
        for (int k = N - 1; k >= 0; k--) {
            float q_tilde[N_STATE];
            for (int i = 0; i < N_STATE; i++) {
                q_tilde[i] = (k == 0) ? ctrl->rho * (ctrl->lambda[i] - ctrl->z[i]) : 0.0f;
            }
            
            // p[k] = C2 * p[k+1] + q_tilde
            float temp[N_STATE];
            mat_vec_mult(temp, ctrl->model.C2, p[k + 1]);
            for (int i = 0; i < N_STATE; i++) {
                p[k][i] = temp[i] + q_tilde[i];
            }
        }

        // 原始更新：使用p[0]计算梯度
        float Bp[N_INPUT] = {0};
        for (int i = 0; i < N_INPUT; i++) {
            for (int j = 0; j < N_STATE; j++) {
                Bp[i] += ctrl->model.B[j][i] * p[0][j]; // B^T * p[0]
            }
        }

        // 计算控制输入u
        for (int i = 0; i < N_INPUT; i++) {
            u[i] = 0.0f;
            // 反馈项
            for (int j = 0; j < N_STATE; j++) {
                u[i] -= ctrl->model.K_inf[i][j] * x_err[j];
            }
            // 前馈项
            for (int j = 0; j < N_INPUT; j++) {
                u[i] -= ctrl->model.C1[i][j] * Bp[j];
            }
        }

        // 松弛更新：投影到约束
        float temp[N_STATE];
        for (int i = 0; i < N_STATE; i++) {
            temp[i] = x[i] + ctrl->lambda[i] / ctrl->rho;
        }
        project_constraints(ctrl->z, temp, ctrl->rho);

        // 对偶更新
        for (int i = 0; i < N_STATE; i++) {
            ctrl->lambda[i] += ctrl->rho * (x[i] - ctrl->z[i]);
        }
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