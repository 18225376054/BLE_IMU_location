#ifndef KALMAN_FILTER2D_H
#define KALMAN_FILTER2D_H


// #define N 4



typedef struct {
    double x;
    double y;
    double vx;
    double vy;
} StateVector2D;

typedef struct {
    double ax;
    double ay;
} ControlVector2D;

typedef struct {
    double dt;
    double process_noise;
    double measurement_noise;
} KalmanFilterParams2D;

// //矩阵相乘   行 列  A 行 列  B C=AB
// void matrixMultiply(int A_rows, int A_cols, int A[A_rows][A_cols], int B_rows, int B_cols, int B[B_rows][B_cols], int C[A_rows][B_cols]);




//矩阵相乘4444   行 列  A 行 列  B C=AB
void matrixMultiply4444(int A_rows, int A_cols, double A[4][4], int B_rows, int B_cols, double B[4][4], double C[4][4]);
//矩阵相乘2444   行 列  A 行 列  B C=AB
void matrixMultiply2444(int A_rows, int A_cols, double A[2][4], int B_rows, int B_cols, double B[4][4], double C[2][4]);
//矩阵相乘2442   行 列  A 行 列  B C=AB
void matrixMultiply2442(int A_rows, int A_cols, double A[2][4], int B_rows, int B_cols, double B[4][2], double C[2][2]);
//矩阵相乘4442   行 列  A 行 列  B C=AB
void matrixMultiply4442(int A_rows, int A_cols, double A[4][4], int B_rows, int B_cols, double B[4][2], double C[4][2]);
//矩阵相乘4222   行 列  A 行 列  B C=AB
void matrixMultiply4222(int A_rows, int A_cols, double A[4][2], int B_rows, int B_cols, double B[2][2], double C[4][2]);

//矩阵转置  result_cols：列  result_rows：行
void matrixTranspose44(double result_cols, double result_rows, double result[4][4], double mat_rows, double mat_cols, double mat[4][4]);
//矩阵转置  result_cols：列  result_rows：行
void matrixTranspose42(double result_cols, double result_rows, double result[4][2], double mat_rows, double mat_cols, double mat[2][4]);

//矩阵求逆  
void matrixInverse2x2(double result[2][2], double mat[2][2]);
// 状态预测协方差矩阵更新
void updateCovariance2D(double P[4][4], double A[4][4], double Q[4][4]);
// 计算卡尔曼增益
void calculateKalmanGain2D(double K[4][2], double P[4][4], double H[2][4], double R[2][2]);


void kalman_filter_init2D(StateVector2D* state, KalmanFilterParams2D* params);

// 卡尔曼滤波更新
//state  状态向量
// control 控制输入向量 uk
//z_x, z_y, z_z 即观测量 蓝牙定位结果
// params卡尔曼滤波参数  包含采样时间  过程噪声 测量噪声
// P 预测协方差矩阵
void kalman_filter_update2D(StateVector2D* state, ControlVector2D control, double z_x, double z_y, KalmanFilterParams2D params, double P[4][4]);

void state_prediction2D(StateVector2D* state, ControlVector2D control, KalmanFilterParams2D params, double P[4][4]);
void state_update2D(StateVector2D* state, double z_x, double z_y, KalmanFilterParams2D params, double P[4][4]);


#endif // KALMAN_FILTER2D_H