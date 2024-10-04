#ifndef IMU_UPDATE_H
#define IMU_UPDATE_H

#include "driver_mpu9250.h"
#include "driver_mpu9250_basic.h"
#include <stdio.h>
#include "gpio.h"
// #include "app_log.h"
#include "driver_mpu9250_dmp.h"
#include <stdint.h>
#include <stdint.h>
#include <math.h>

typedef struct{
    double w, x, y, z;
}Quaternion;

typedef struct {
    double x, y, z;
}Vector3;


uint8_t mpu_init(void);
uint8_t readIMUinit();
uint8_t readIMUacc(Vector3 *acc,Quaternion *q);
void a_receive_callback(uint8_t type);
void a_dmp_tap_callback(uint8_t count, uint8_t direction);
void a_dmp_orient_callback(uint8_t orientation);



// 四元数与向量相乘，旋转向量
Vector3 quat_rotate(Quaternion q, Vector3 v);

// 向量加法
Vector3 vec_add(Vector3 a, Vector3 b);

// 向量减法
Vector3 vec_sub(Vector3 a, Vector3 b);

// 向量标量乘法
Vector3 vec_scale(Vector3 v, double s);

// 位置更新函数
void update_position(Quaternion q, Vector3 a_b, Vector3* v, Vector3* p, Vector3 g, double deltaT);
//转换为全球坐标系下的加速度 并减去重力加速度g
void update_acc(Quaternion q, Vector3 a_b,Vector3 g, Vector3* acc);
#endif