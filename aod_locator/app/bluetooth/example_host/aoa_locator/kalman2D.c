#include "kalman2D.h"

// #define N 4

// 初始化卡尔曼滤波状态  以及系统初始状态
void kalman_filter_init2D(StateVector2D* state, KalmanFilterParams2D* params) {
//协方差预测矩阵 P0时刻
// static double P[4][4] = {
//         {1, 0, 0, 0},
//         {0, 1, 0, 0},
//         {0, 0, 1, 0},
//         {0, 0, 0, 1}
//     };
    
    state->x = -0.5;
    state->y = 2;
    state->vx = 0;
    state->vy = 0;
    params->dt=0.8;
    params->process_noise=0.1;
    params->measurement_noise=0.2;

}



void state_prediction2D(StateVector2D* state, ControlVector2D control, KalmanFilterParams2D params, double P[4][4]) {
    // 状态预测
    state->x += state->vx * params.dt + 0.5 * control.ax * params.dt * params.dt;
    state->y += state->vy * params.dt + 0.5 * control.ay * params.dt * params.dt;
    state->vx += control.ax * params.dt;
    state->vy += control.ay * params.dt;

    // 状态转移矩阵 A
    double A[4][4]={
        {1, 0, params.dt, 0},
        {0, 1, 0, params.dt},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    // 过程噪声协方差矩阵 Q
    double Q[4][4] = {
        {params.process_noise, 0, 0, 0},
        {0, params.process_noise, 0, 0},
        {0, 0, params.process_noise, 0},
        {0, 0, 0, params.process_noise}
    };

    // 更新预测协方差矩阵 P
    updateCovariance2D(P, A, Q);
}


void state_update2D(StateVector2D* state, double z_x, double z_y, KalmanFilterParams2D params, double P[4][4]) {
    // 观测矩阵 H
    double H[2][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0}
    };

    // 观测噪声协方差矩阵 R
    double R[2][2] = {
        {params.measurement_noise, 0},
        {0, params.measurement_noise}
    };

    // 计算卡尔曼增益
    double K[4][2];
    calculateKalmanGain2D(K, P, H, R);

    // 更新状态估计
    double y[2] = {z_x - state->x, z_y - state->y};
    state->x += K[0][0] * y[0] + K[0][1] * y[1];
    state->y += K[1][0] * y[0] + K[1][1] * y[1];
    state->vx += K[2][0] * y[0] + K[2][1] * y[1];
    state->vy += K[3][0] * y[0] + K[3][1] * y[1];

    // 更新协方差矩阵 P
    double temp_KH[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 2; ++k) {
                temp_KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    double I[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        I[i][i] = 1.0;
    }

    double IKH[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            IKH[i][j] = I[i][j] - temp_KH[i][j];
        }
    }

    double temp_P[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                temp_P[i][j] += IKH[i][k] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            P[i][j] = temp_P[i][j];
        }
    }
}







void kalman_filter_update2D(StateVector2D* state, ControlVector2D control, double z_x, double z_y, KalmanFilterParams2D params, double P[4][4]) {
    // 状态预测
    state->x += state->vx * params.dt + 0.5 * control.ax * params.dt * params.dt;
    state->y += state->vy * params.dt + 0.5 * control.ay * params.dt * params.dt;
    state->vx += control.ax * params.dt;
    state->vy += control.ay * params.dt;

//状态转移矩阵A
double A[4][4]={
{1, 0, params.dt, 0},
{0, 1, 0, params.dt},
{0, 0, 1, 0},
{0, 0, 0, 1}
};
// 状态预测协方差矩阵
double Q[4][4] = {
        {params.process_noise, 0, 0, 0},
        {0, params.process_noise, 0, 0},
        {0, 0, params.process_noise, 0},
        {0, 0, 0, params.process_noise}
    };

// 更新预测协方差矩阵 P
updateCovariance2D(P, A, Q);

// 观测矩阵
 double H[2][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0}
    };
// 观测噪声协方差矩阵
double R[2][2] = {
        {params.measurement_noise, 0},
        {0, params.measurement_noise}
    };

// 计算卡尔曼增益
    double K[4][2];
    calculateKalmanGain2D(K, P, H, R);

    // 更新状态估计 
    // 这里的state->x是状态预测值
    double y[2] = {z_x - state->x, z_y - state->y};
    // 这里进行状态更新 state->x 为状态估计值
    state->x += K[0][0] * y[0] + K[0][1] * y[1];
    state->y += K[1][0] * y[0] + K[1][1] * y[1];
    state->vx += K[2][0] * y[0] + K[2][1] * y[1];
    state->vy += K[3][0] * y[0] + K[3][1] * y[1];

    // 更新协方差矩阵
    double temp_KH[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 2; ++k) {
                temp_KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    double I[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        I[i][i] = 1.0;
    }

    double IKH[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            IKH[i][j] = I[i][j] - temp_KH[i][j];
        }
    }

    double temp_P[4][4] = {0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                temp_P[i][j] += IKH[i][k] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            P[i][j] = temp_P[i][j];
        }
    }
}



// //矩阵相乘   行 列  A 行 列  B C=AB
// void matrixMultiply(int A_rows, int A_cols, int A[A_rows][A_cols], int B_rows, int B_cols, int B[B_rows][B_cols], int C[A_rows][B_cols]) {
//     int i, j, k;

//     // 初始化结果矩阵C
//     for(i = 0; i < A_rows; i++) {
//         for(j = 0; j < B_cols; j++) {
//             C[i][j] = 0;
//         }
//     }

//     // 计算乘积
//     for(i = 0; i < A_rows; i++) {
//         for(j = 0; j < B_cols; j++) {
//             for(k = 0; k < A_cols; k++) {
//                 C[i][j] += A[i][k] * B[k][j];
//             }
//         }
//     }
// }



//矩阵相乘4444   行 列  A 行 列  B C=AB
void matrixMultiply4444(int A_rows, int A_cols, double A[4][4], int B_rows, int B_cols, double B[4][4], double C[4][4]) {
    int i, j, k;

    // 初始化结果矩阵C
    for(i = 0; i < A_rows; i++) {
        for(j = 0; j < B_cols; j++) {
            C[i][j] = 0;
        }
    }

    // 计算乘积
    for(i = 0; i < A_rows; i++) {
        for(j = 0; j < B_cols; j++) {
            for(k = 0; k < A_cols; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


//矩阵相乘2444   行 列  A 行 列  B C=AB
void matrixMultiply2444(int A_rows, int A_cols, double A[2][4], int B_rows, int B_cols, double B[4][4], double C[2][4]) {
    int i, j, k;

    // 初始化结果矩阵C
    for(i = 0; i < A_rows; i++) {
        for(j = 0; j < B_cols; j++) {
            C[i][j] = 0;
        }
    }

    // 计算乘积
    for(i = 0; i < A_rows; i++) {
        for(j = 0; j < B_cols; j++) {
            for(k = 0; k < A_cols; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

//矩阵相乘2442   行 列  A 行 列  B C=AB
void matrixMultiply2442(int A_rows, int A_cols, double A[2][4], int B_rows, int B_cols, double B[4][2], double C[2][2]) {
    int i, j, k;

    // 初始化结果矩阵C
    for(i = 0; i < A_rows; i++) {
        for(j = 0; j < B_cols; j++) {
            C[i][j] = 0;
        }
    }

    // 计算乘积
    for(i = 0; i < A_rows; i++) {
        for(j = 0; j < B_cols; j++) {
            for(k = 0; k < A_cols; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


//矩阵相乘4442   行 列  A 行 列  B C=AB
void matrixMultiply4442(int A_rows, int A_cols, double A[4][4], int B_rows, int B_cols, double B[4][2], double C[4][2]) {
    int i, j, k;

    // 初始化结果矩阵C
    for(i = 0; i < A_rows; i++) {
        for(j = 0; j < B_cols; j++) {
            C[i][j] = 0;
        }
    }

    // 计算乘积
    for(i = 0; i < A_rows; i++) {
        for(j = 0; j < B_cols; j++) {
            for(k = 0; k < A_cols; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


//矩阵相乘4222   行 列  A 行 列  B C=AB
void matrixMultiply4222(int A_rows, int A_cols, double A[4][2], int B_rows, int B_cols, double B[2][2], double C[4][2]) {
    int i, j, k;

    // 初始化结果矩阵C
    for(i = 0; i < A_rows; i++) {
        for(j = 0; j < B_cols; j++) {
            C[i][j] = 0;
        }
    }

    // 计算乘积
    for(i = 0; i < A_rows; i++) {
        for(j = 0; j < B_cols; j++) {
            for(k = 0; k < A_cols; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}
 
//矩阵转置  result_cols：列  result_rows：行
void matrixTranspose44(double result_cols, double result_rows, double result[4][4], double mat_rows, double mat_cols, double mat[4][4]) {
    for (int i = 0; i < result_cols; ++i) {
        for (int j = 0; j < result_rows; ++j) {
            result[j][i] = mat[i][j];
        }
    }
}

//矩阵转置  result_cols：列  result_rows：行
// void matrixTranspose42(double result_cols, double result_rows, double result[4][2], double mat_rows, double mat_cols, double mat[2][4]) {
//     for (int i = 0; i < result_cols; ++i) {
//         for (int j = 0; j < result_rows; ++j) {
//             result[j][i] = mat[i][j];
//         }
//     }
// }
void matrixTranspose42(double result_cols, double result_rows, double result[4][2], double mat_rows, double mat_cols, double mat[2][4]) {
    for (int i = 0; i < result_cols; i++) {  // Transpose loops should use result_rows and result_cols
        for (int j = 0; j < result_rows; j++) {
            result[i][j] = mat[j][i];
        }
    }
}


//矩阵求逆  
void matrixInverse2x2(double result[2][2], double mat[2][2]) {
    double det = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
    double invdet = 1.0 / det;

    result[0][0] = mat[1][1] * invdet;
    result[0][1] = -mat[0][1] * invdet;
    result[1][0] = -mat[1][0] * invdet;
    result[1][1] = mat[0][0] * invdet;
}






// 状态预测协方差矩阵更新
void updateCovariance2D(double P[4][4], double A[4][4], double Q[4][4]) {
    double AP[4][4];
    double APT[4][4];
    double A_transpose[4][4];

    // 计算AP
    matrixMultiply4444(4,4,A,4,4,P,AP);

    // 计算A的转置 列 行 转置后的矩阵 行 列 待转换矩阵
    matrixTranspose44(4,4,A_transpose,4,4,A);

    // 计算AP * A^T
    matrixMultiply4444(4,4,AP,4,4,A_transpose, APT);
    // P = APT + Q
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            P[i][j] = APT[i][j] + Q[i][j];
        }
    }
}



// 计算卡尔曼增益
void calculateKalmanGain2D(double K[4][2], double P[4][4], double H[2][4], double R[2][2]) {

    double HT[4][2];
    double HP[2][4];
    double S[2][2];
    double S_inv[2][2];
    double PHT[4][2];
 
    // 计算H的转置HT 列 行 转置后的矩阵 行 列 待转换矩阵
    matrixTranspose42(4,2,HT,2,4,H);

    // 计算s=HPH^T  matrixMultiply 行 列  A 行 列  B C=AB
    matrixMultiply2444(2,4,H,4,4,P,HP);
    matrixMultiply2442(2,4,HP,4,2,HT,S);

    // S = HPH^T + R
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            S[i][j] += R[i][j];
        }
    }
    // 计算S的逆
    matrixInverse2x2(S_inv, S);

    // 计算卡尔曼增益 K = PHTS^-1
    matrixMultiply4442(4,4,P,4,2,HT,PHT);
    matrixMultiply4222(4,2,PHT,2,2,S_inv,K);
}