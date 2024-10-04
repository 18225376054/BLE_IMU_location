#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct {
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
} StateVector;

typedef struct {
    double ax;
    double ay;
    double az;
} ControlVector;

typedef struct {
    double dt;
    double process_noise;
    double measurement_noise;
} KalmanFilterParams;


//deltaT 采样时间
#define SAMPLET 0.8
// 矩阵的维度
// #define N 6
// #define M 3

//预测协方差
void matrixMultiply(double result[6][6], double mat1[6][6], double mat2[6][6]);
void matrixTranspose(double result[6][6], double mat[6][6]);


//计算卡尔曼增益

void matrixTranspose3D(double result[6][3], double mat[3][6], int rows, int cols);
void matrixInverse3x3(double result[3][3], double mat[3][3]);
void matrixMultiply3666(double result[3][6], double mat1[3][6], double mat2[6][6]);
void matrixMultiply3663(double result[3][3], double mat1[3][6], double mat2[6][3]);
void matrixMultiply6663(double result[6][3], double mat1[6][6], double mat2[6][3]);
void matrixMultiply6333(double result[6][3], double mat1[6][3], double mat2[3][3]);

void calculateKalmanGain(double K[6][3], double P[6][6], double H[3][6], double R[3][3]);


void updateCovariance(double P[6][6], double A[6][6], double Q[6][6]);
void kalman_filter_init(StateVector* state, KalmanFilterParams* params);
void kalman_filter_update(StateVector* state, ControlVector control, double z_x, double z_y, double z_z, KalmanFilterParams params);
//卡尔曼预测
void kalman_filter_predict3d(StateVector* state, ControlVector control, KalmanFilterParams params, double P[6][6]);
//卡尔曼更新
void kalman_only_update3d(StateVector* state, double z_x, double z_y, double z_z, KalmanFilterParams params, double P[6][6]);
#endif // KALMAN_FILTER_H
