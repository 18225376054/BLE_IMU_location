// 2024.5.27 jieryyyyy
#include "kalman.h"
#include <stdio.h>
#include <math.h>


//deltaT 采样时间
#define SAMPLET 0.8
// 矩阵的维度
// #define N 6
// #define M 3


//协方差预测矩阵 P0时刻
static double P3D[6][6] = {
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}
    };



// 卡尔曼增益K
double K[6][3]={0};

// 初始化卡尔曼滤波状态
void kalman_filter_init(StateVector* state, KalmanFilterParams* params) {

    //初始位置和速度
    state->x = 0;
    state->y = 0;
    state->z = 0;
    state->vx = 0;
    state->vy = 0;
    state->vz = 0;
    params->dt=SAMPLET;
    params->process_noise=0.5;
    params->measurement_noise=1;

}






//卡尔曼预测
void kalman_filter_predict3d(StateVector* state, ControlVector control, KalmanFilterParams params, double P[6][6]) {
    // 状态预测
    state->x += state->vx * params.dt + 0.5 * control.ax * params.dt * params.dt;
    state->y += state->vy * params.dt + 0.5 * control.ay * params.dt * params.dt;
    state->z += state->vz * params.dt + 0.5 * control.az * params.dt * params.dt;
    state->vx += control.ax * params.dt;
    state->vy += control.ay * params.dt;
    state->vz += control.az * params.dt;

    // 状态转移矩阵A
    double A[6][6] = {
        {1, 0, 0, params.dt, 0, 0},
        {0, 1, 0, 0, params.dt, 0},
        {0, 0, 1, 0, 0, params.dt},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1},
    };

    // 状态预测协方差矩阵Q
    double Q[6][6] = {
        {params.process_noise, 0, 0, 0, 0, 0},
        {0, params.process_noise, 0, 0, 0, 0},
        {0, 0, params.process_noise, 0, 0, 0},
        {0, 0, 0, params.process_noise, 0, 0},
        {0, 0, 0, 0, params.process_noise, 0},
        {0, 0, 0, 0, 0, params.process_noise},
    };

    // 更新预测协方差矩阵 P
    updateCovariance(P, A, Q);
}

//卡尔曼更新 只有卡尔曼滤波的后面三个等式 即更新部分
void kalman_only_update3d(StateVector* state, double z_x, double z_y, double z_z, KalmanFilterParams params, double P[6][6]) {
    // 观测矩阵H
    double H[3][6] = {
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0}
    };

    // 观测噪声协方差矩阵R
    double R[3][3] = {
        {params.measurement_noise, 0, 0},
        {0, params.measurement_noise, 0},
        {0, 0, params.measurement_noise}
    };

    // 计算卡尔曼增益K
    double K[6][3];
    calculateKalmanGain(K, P, H, R);

    // 更新状态估计
    double y[3] = {z_x - state->x, z_y - state->y, z_z - state->z};
    state->x += K[0][0] * y[0] + K[0][1] * y[1] + K[0][2] * y[2];
    state->y += K[1][0] * y[0] + K[1][1] * y[1] + K[1][2] * y[2];
    state->z += K[2][0] * y[0] + K[2][1] * y[1] + K[2][2] * y[2];
    state->vx += K[3][0] * y[0] + K[3][1] * y[1] + K[3][2] * y[2];
    state->vy += K[4][0] * y[0] + K[4][1] * y[1] + K[4][2] * y[2];
    state->vz += K[5][0] * y[0] + K[5][1] * y[1] + K[5][2] * y[2];

    // 更新协方差矩阵P
    double temp_KH[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 3; ++k) {
                temp_KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    double I[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        I[i][i] = 1.0;
    }

    double IKH[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            IKH[i][j] = I[i][j] - temp_KH[i][j];
        }
    }

    double temp_P[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                temp_P[i][j] += IKH[i][k] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            P[i][j] = temp_P[i][j];
        }
    }
}







// 卡尔曼滤波更新
//state  状态向量
// control 控制输入向量 uk
//z_x, z_y, z_z 即观测量 蓝牙定位结果
// params卡尔曼滤波参数  包含采样时间  过程噪声 测量噪声
// P 预测协方差矩阵
void kalman_filter_update(StateVector* state, ControlVector control, double z_x, double z_y, double z_z, KalmanFilterParams params) {
    // 状态预测
    state->x += state->vx * params.dt + 0.5 * control.ax * params.dt * params.dt;
    state->y += state->vy * params.dt + 0.5 * control.ay * params.dt * params.dt;
    state->z += state->vz * params.dt + 0.5 * control.az * params.dt * params.dt;
    state->vx += control.ax * params.dt;
    state->vy += control.ay * params.dt;
    state->vz += control.az * params.dt;

//状态转移矩阵A
double A[6][6]={
{1,0,0,params.dt,0,0},
{0,1,0,0,params.dt,0},
{0,0,1,0,0,params.dt},
{0,0,0,1,0,0},
{0,0,0,0,1,0},
{0,0,0,0,0,1},
};
// 状态预测协方差矩阵
double Q[6][6] = {
        {params.process_noise, 0, 0, 0, 0, 0},
        {0, params.process_noise, 0, 0, 0, 0},
        {0, 0, params.process_noise, 0, 0, 0},
        {0, 0, 0, params.process_noise, 0, 0},
        {0, 0, 0, 0, params.process_noise, 0},
        {0, 0, 0, 0, 0, params.process_noise}
    };


//更新预测协方差矩阵  P
updateCovariance(P3D,A,Q);




// 观测矩阵
 double H[3][6] = {
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0}
    };
// 观测噪声协方差矩阵
double R[3][3] = {
        {params.measurement_noise, 0, 0},
        {0, params.measurement_noise, 0},
        {0, 0, params.measurement_noise}
    };


// 计算卡尔曼增益
    calculateKalmanGain(K, P3D, H, R);

    // 更新状态估计 
    //这里的state->x是状态预测值
    double y[3] = {z_x - state->x, z_y - state->y, z_z - state->z};
    //这里进行状态更新  state->x 为状态估计值
    state->x += K[0][0] * y[0] + K[0][1] * y[1] + K[0][2] * y[2];
    state->y += K[1][0] * y[0] + K[1][1] * y[1] + K[1][2] * y[2];
    state->z += K[2][0] * y[0] + K[2][1] * y[1] + K[2][2] * y[2];
    state->vx += K[3][0] * y[0] + K[3][1] * y[1] + K[3][2] * y[2];
    state->vy += K[4][0] * y[0] + K[4][1] * y[1] + K[4][2] * y[2];
    state->vz += K[5][0] * y[0] + K[5][1] * y[1] + K[5][2] * y[2];

    // 更新协方差矩阵
    double temp_KH[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 3; ++k) {
                temp_KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    double I[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        I[i][i] = 1.0;
    }

    double IKH[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            IKH[i][j] = I[i][j] - temp_KH[i][j];
        }
    }

    double temp_P[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                temp_P[i][j] += IKH[i][k] * P3D[k][j];
            }
        }
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            P3D[i][j] = temp_P[i][j];
        }
    }
}





// 矩阵乘法
void matrixMultiply(double result[6][6], double mat1[6][6], double mat2[6][6]) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < 6; ++k) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

// 矩阵转置    
void matrixTranspose(double result[6][6], double mat[6][6]) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            result[i][j] = mat[j][i];
        }
    }
}



// 矩阵乘法函数 3x6 的矩阵 mat1 与一个 6x6 的矩阵 mat2 相乘
void matrixMultiply3666(double result[3][6], double mat1[3][6], double mat2[6][6]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < 6; ++k) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

// 矩阵乘法函数 3x6 的矩阵 mat1 与一个 6x3 的矩阵 mat2 相乘
void matrixMultiply3663(double result[3][3], double mat1[3][6], double mat2[6][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < 6; ++k) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}



// 矩阵乘法函数 6x6 的矩阵 mat1 与一个 6x3 的矩阵 mat2 相乘
void matrixMultiply6663(double result[6][3], double mat1[6][6], double mat2[6][3]) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < 6; ++k) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}



void matrixMultiply6333(double result[6][3], double mat1[6][3], double mat2[3][3]) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}


// 矩阵转置
void matrixTranspose3D(double result[6][3], double mat[3][6], int rows, int cols) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[j][i] = mat[i][j];
        }
    }
}

// 矩阵求逆（假设为3x3矩阵）
void matrixInverse3x3(double result[3][3], double mat[3][3]) {
    double det = mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) -
                 mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
                 mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);

    double invdet = 1.0 / det;

    result[0][0] = (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) * invdet;
    result[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) * invdet;
    result[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) * invdet;
    result[1][0] = (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]) * invdet;
    result[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) * invdet;
    result[1][2] = (mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2]) * invdet;
    result[2][0] = (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]) * invdet;
    result[2][1] = (mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1]) * invdet;
    result[2][2] = (mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1]) * invdet;
}



// 状态预测协方差矩阵更新
void updateCovariance(double P[6][6], double A[6][6], double Q[6][6]) {
    double AP[6][6];
    double APT[6][6];
    double A_transpose[6][6];

    // 计算AP
    matrixMultiply(AP, A, P);

    // 计算A的转置
    matrixTranspose(A_transpose, A);

    // 计算AP * A^T
    matrixMultiply(APT, AP, A_transpose);

    // P = APT + Q
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            P[i][j] = APT[i][j] + Q[i][j];
        }
    }
}



// 计算卡尔曼增益
void calculateKalmanGain(double K[6][3], double P[6][6], double H[3][6], double R[3][3]) {
    double HT[6][3];
    double HP[3][6];
    double S[3][3];
    double S_inv[3][3];
    double PHT[6][3];
    // 计算H的转置HT
    matrixTranspose3D(HT, H, 3, 6);

    // 计算s=HPH^T
    matrixMultiply3666(HP, H, P);
    matrixMultiply3663(S, HP, HT);

    // S = HPH^T + R
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            S[i][j] += R[i][j];
        }
    }
    // 计算S的逆
    matrixInverse3x3(S_inv, S);

    // 计算卡尔曼增益 K = PHTS^-1

    matrixMultiply6663(PHT, P, HT);
    matrixMultiply6333(K, PHT, S_inv);
}




