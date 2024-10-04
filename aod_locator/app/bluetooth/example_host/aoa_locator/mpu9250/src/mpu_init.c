//mpu初始化
#include "mpu_init.h"




#define app_log_info printf
// float acc[3];
// float gyro[3];
// float mag[3];



static int16_t gs_accel_raw[128][3];      /**< accel raw buffer */
static float gs_accel_g[128][3];          /**< accel g buffer */
static int16_t gs_gyro_raw[128][3];       /**< gyro raw buffer */
static float gs_gyro_dps[128][3];         /**< gyro dps buffer */
static int32_t gs_quat[128][4];           /**< quat buffer */
static float gs_pitch[128];               /**< pitch buffer */
static float gs_roll[128];                /**< roll buffer */
static float gs_yaw[128];                 /**< yaw buffer */



// typedef struct{
//     double w, x, y, z;
// }Quaternion;

// typedef struct {
//     double x, y, z;
// }Vector3;
// 初始化 Quaternion 结构体变量

static  Quaternion q = {0, 0, 0, 0}; // 假设初始化为单位四元数
// 初始化 Vector3 结构体变量
static Vector3 a_b = {1.0, 0.0, 0.0}; // 本体坐标系下的加速度向量
static Vector3 v = {0.0, 0.0, 0.0}; // 速度向量
static Vector3 p = {0.0, 0.0, 0.0}; // 位置向量
static Vector3 g={0.0, 0.0, 9.8}; // 全球坐标系下的重力加速度
static double deltaT=0.08; // 采样时间 80ms
// static Vector3 acc_bias={-0.25, -0.15, -0.25}; // 采样偏差
static Vector3 acc_bias={-0.2, -0.1, -0.25}; // 采样偏差




uint8_t (*g_gpio_irq)(void) = NULL;        /**< gpio irq */
int receive_Count=0;
int dmp_tap_Count=0;
int dmp_orient_Count=0;
mpu9250_address_t addr;	
uint16_t len=128;

// uint8_t mpu_init(void);


//初始化并读取MPU数据  成功返回0
uint8_t mpu_init(void)
{
    uint8_t res = 0;
    // uint8_t prev = 0;
    mpu9250_interface_t interface = MPU9250_INTERFACE_IIC;
    res=mpu9250_basic_init(interface, MPU9250_ADDRESS_AD0_LOW); //MPU初始化
    if (res != 0)                                                                   /* check the result */
    {
        // app_log_info("mpu9250_basic_init failed");
        printf("mpu9250_basic_init failed\n");
        // handle->debug_print("mpu9250: read who am i failed.\n");                    /* read who am I failed */
        // (void)a_mpu9250_deinit(handle);                                             /* iic or spi deinit */        
        return 5;                                                                   /* return error */
    }


//读取dmp数据
        if (gpio_interrupt_init() != 0)
        {
            mpu9250_interface_debug_print("gpio_interrupt_init failed\n");
            return 1;
        }
        g_gpio_irq = mpu9250_dmp_irq_handler;
        if (mpu9250_dmp_init(MPU9250_INTERFACE_IIC, MPU9250_ADDRESS_AD0_LOW, a_receive_callback,
                     a_dmp_tap_callback, a_dmp_orient_callback) != 0)
        {
            g_gpio_irq = NULL;
            gpio_interrupt_deinit();
            mpu9250_interface_debug_print("mpu9250_dmp_init failed\n");
            return 1;
        }

        mpu9250_interface_delay_ms(500);
        /* gpio init */

        len=128;
        while(1)
        {
        if ((res=mpu9250_dmp_read_all(gs_accel_raw, gs_accel_g,
                                    gs_gyro_raw, gs_gyro_dps,
                                    gs_quat,
                                    gs_pitch, gs_roll, gs_yaw,
                                    &len) != 0))
            {
                app_log_info("mpu9250_dmp_read_all failed. res is:%d \n",res);
                mpu9250_dmp_deinit();
                g_gpio_irq = NULL;
                gpio_interrupt_deinit();

                return 1;
            }
        // app_log_info("len is:%d \n",gs_quat[0][0],len);
        // app_log_info("gs_accel_raw 0:%d 1:%d 2:%d,gs_accel_g 0:%f,1:%f,1:%f\n",gs_accel_raw[0][0],gs_accel_raw[0][1],gs_accel_raw[0][2],gs_accel_g[0][0],gs_accel_g[0][1],gs_accel_g[0][2]);
        // app_log_info("姿态角 俯仰角:%f,横滚角:%f,偏航角:%f\n",gs_pitch[0],gs_roll[0],gs_yaw[0]);


        // app_log_info("放大的四元数 0:%d,1:%d,2:%d,3:%d\n",gs_quat[0][0],gs_quat[0][1],gs_quat[0][2],gs_quat[0][3]);

    // q.w=gs_quat[0][0];
    // q.x=gs_quat[0][1];
    // q.y=gs_quat[0][2];
    // q.z=gs_quat[0][3];
    a_b.x=(gs_accel_g[0][0]*9.8)-acc_bias.x;//这里的加速度的单位是g  需要乘以9.8 则单位为m/s2
    a_b.y=(gs_accel_g[0][1]*9.8)-acc_bias.y;
    a_b.z=(gs_accel_g[0][2]*9.8)-acc_bias.z;
    q.w=gs_quat[0][0]/1073741824.0f; //     四元数被放大了2^30倍  真正的四元数应该是 （-1，1）之间。
    q.x=gs_quat[0][1]/1073741824.0f;
    q.y=gs_quat[0][2]/1073741824.0f;
    q.z=gs_quat[0][3]/1073741824.0f;

    app_log_info("加速度 X:%f,Y:%f,Z:%f\n",a_b.x,a_b.y,a_b.z);
    app_log_info("四元数 0:%f,1:%f,2:%f,3:%f\n",q.w,q.x,q.y,q.z);


    // 调用位置更新函数
    update_position(q, a_b, &v, &p, g, deltaT);
    // printf("position x:%f y:%f z:%f \n",p.x,p.y,p.z);
    printf("\x1b[31mposition x:%f y:%f z:%f \x1b[0m\n", p.x, p.y, p.z); //打印为红色
    mpu9250_interface_delay_ms(80);   //采样频率50HZ时  按道理来说应该20MS 但太小了会导致FIFO没有数据可读，太大了会导致FIFO数据溢出 80ms时还行  但运行时间久了还是会溢出
        }
	
	

        // /* gpio deinit */
        // g_gpio_irq = NULL;
        // (void)gpio_interrupt_deinit();



return 0;

}

// uint8_t readIMUacc(Vector3 *acc,Quaternion *q)
// {
//     uint8_t res = 0;
//     // uint8_t prev = 0;

//     mpu9250_interface_t interface = MPU9250_INTERFACE_IIC;
//     res=mpu9250_basic_init(interface, MPU9250_ADDRESS_AD0_LOW); //MPU初始化
//     if (res != 0)                                                                   /* check the result */
//     {
//         // app_log_info("mpu9250_basic_init failed");
//         printf("mpu9250_basic_init failed\n");
//         // handle->debug_print("mpu9250: read who am i failed.\n");                    /* read who am I failed */
//         // (void)a_mpu9250_deinit(handle);                                             /* iic or spi deinit */        
//         return 5;                                                                   /* return error */
//     }

// // /读取dmp数据
//         if (gpio_interrupt_init() != 0)
//         {
//             mpu9250_interface_debug_print("gpio_interrupt_init failed\n");
//             return 1;
//         }
//         g_gpio_irq = mpu9250_dmp_irq_handler;
//         if (mpu9250_dmp_init(MPU9250_INTERFACE_IIC, MPU9250_ADDRESS_AD0_LOW, a_receive_callback,
//                      a_dmp_tap_callback, a_dmp_orient_callback) != 0)
//         {
//             g_gpio_irq = NULL;
//             gpio_interrupt_deinit();
//             mpu9250_interface_debug_print("mpu9250_dmp_init failed\n");
//             return 1;
//         }

//         mpu9250_interface_delay_ms(500);
//         /* gpio init */



//         if ((res=mpu9250_dmp_read_all(gs_accel_raw, gs_accel_g,
//                                     gs_gyro_raw, gs_gyro_dps,
//                                     gs_quat,
//                                     gs_pitch, gs_roll, gs_yaw,
//                                     &len) != 0))
//             {
//                 app_log_info("mpu9250_dmp_read_all failed. res is:%d \n",res);
//                 mpu9250_dmp_deinit();
//                 g_gpio_irq = NULL;
//                 gpio_interrupt_deinit();

//                 return 1;
//             }
//     acc->x=(gs_accel_g[0][0]*9.8)-acc_bias.x;//这里的加速度的单位是g  需要乘以9.8 则单位为m/s2
//     acc->y=(gs_accel_g[0][1]*9.8)-acc_bias.y;
//     acc->z=(gs_accel_g[0][2]*9.8)-acc_bias.z;
//     q->w=gs_quat[0][0]/1073741824.0f; //     四元数被放大了2^30倍  真正的四元数应该是 （-1，1）之间。
//     q->x=gs_quat[0][1]/1073741824.0f;
//     q->y=gs_quat[0][2]/1073741824.0f;
//     q->z=gs_quat[0][3]/1073741824.0f;
//     app_log_info("加速度 X:%f,Y:%f,Z:%f\n",acc->x,acc->y,acc->z);
//     app_log_info("四元数 0:%f,1:%f,2:%f,3:%f\n",q->w,q->x,q->y,q->z);
// return 0;

    


//     // // 调用位置更新函数
//     // update_position(q, a_b, &v, &p, g, deltaT);
//     // // printf("position x:%f y:%f z:%f \n",p.x,p.y,p.z);
//     // printf("\x1b[31mposition x:%f y:%f z:%f \x1b[0m\n", p.x, p.y, p.z); //打印为红色
//     // mpu9250_interface_delay_ms(80);   //采样频率50HZ时  按道理来说应该20MS 但太小了会导致FIFO没有数据可读，太大了会导致FIFO数据溢出 80ms时还行  但运行时间久了还是会溢出


// }


uint8_t readIMUinit()
{
uint8_t res = 0;
    // uint8_t prev = 0;

    mpu9250_interface_t interface = MPU9250_INTERFACE_IIC;
    res=mpu9250_basic_init(interface, MPU9250_ADDRESS_AD0_LOW); //MPU初始化
    if (res != 0)                                                                   /* check the result */
    {
        // app_log_info("mpu9250_basic_init failed");
        printf("mpu9250_basic_init failed\n");
        // handle->debug_print("mpu9250: read who am i failed.\n");                    /* read who am I failed */
        // (void)a_mpu9250_deinit(handle);                                             /* iic or spi deinit */        
        return 5;                                                                   /* return error */
    }
printf("mpu9250_basic_init success\n");
// /读取dmp数据
        if (gpio_interrupt_init() != 0)
        {
            mpu9250_interface_debug_print("gpio_interrupt_init failed\n");
            return 1;
        }
        g_gpio_irq = mpu9250_dmp_irq_handler;
        if (mpu9250_dmp_init(MPU9250_INTERFACE_IIC, MPU9250_ADDRESS_AD0_LOW, a_receive_callback,
                     a_dmp_tap_callback, a_dmp_orient_callback) != 0)
        {
            g_gpio_irq = NULL;
            gpio_interrupt_deinit();
            mpu9250_interface_debug_print("mpu9250_dmp_init failed\n");
            return 1;
        }

    mpu9250_interface_delay_ms(500);
 printf("read imu init success\n");
        /* gpio init */
return 0;
}

uint8_t readIMUacc(Vector3 *acc,Quaternion *q)
{
    uint8_t res = 0;
    uint16_t len=128;
res=mpu9250_dmp_read_all(gs_accel_raw, gs_accel_g,
                                    gs_gyro_raw, gs_gyro_dps,
                                    gs_quat,
                                    gs_pitch, gs_roll, gs_yaw,
                                    &len);
if(res!= 0)
    {
        app_log_info("mpu9250_dmp_read_all failed. res is:%d \n",res);
        mpu9250_dmp_deinit();
        g_gpio_irq = NULL;
        gpio_interrupt_deinit();
        return 1;
    }
    acc->x=(gs_accel_g[0][0]*9.8)-acc_bias.x;//这里的加速度的单位是g  需要乘以9.8 则单位为m/s2
    acc->y=(gs_accel_g[0][1]*9.8)-acc_bias.y;
    acc->z=(gs_accel_g[0][2]*9.8)-acc_bias.z;
    // q->w=gs_quat[0][0]/1073741824.0f; //     四元数被放大了2^30倍  真正的四元数应该是 （-1，1）之间。
    // q->x=gs_quat[0][1]/1073741824.0f;
    // q->y=gs_quat[0][2]/1073741824.0f;
    // q->z=gs_quat[0][3]/1073741824.0f;
    app_log_info("加速度 X:%f,Y:%f,Z:%f\n",acc->x,acc->y,acc->z);
    // app_log_info("四元数 0:%f,1:%f,2:%f,3:%f\n",q->w,q->x,q->y,q->z);
return 0;    
    // // 调用位置更新函数
    // update_position(q, a_b, &v, &p, g, deltaT);
    // // printf("position x:%f y:%f z:%f \n",p.x,p.y,p.z);
    // printf("\x1b[31mposition x:%f y:%f z:%f \x1b[0m\n", p.x, p.y, p.z); //打印为红色
    // mpu9250_interface_delay_ms(80);   //采样频率50HZ时  按道理来说应该20MS 但太小了会导致FIFO没有数据可读，太大了会导致FIFO数据溢出 80ms时还行  但运行时间久了还是会溢出
}





void a_receive_callback(uint8_t type)
	 {
        		receive_Count++;
switch (type)
    {
        case MPU9250_INTERRUPT_MOTION :
        {
            mpu9250_interface_debug_print("mpu9250: irq motion.\n");
            
            break;
        }
        case MPU9250_INTERRUPT_FIFO_OVERFLOW :
        {
            mpu9250_interface_debug_print("mpu9250: irq fifo overflow.\n");
            
            break;
        }
        case MPU9250_INTERRUPT_FSYNC_INT :
        {
            mpu9250_interface_debug_print("mpu9250: irq fsync int.\n");
            
            break;
        }
        case MPU9250_INTERRUPT_DMP :
        {
            mpu9250_interface_debug_print("mpu9250: irq dmp\n");
            
            break;
        }
        case MPU9250_INTERRUPT_DATA_READY :
        {
            mpu9250_interface_debug_print("mpu9250: irq data ready\n");
            
            break;
        }
        default :
        {
            mpu9250_interface_debug_print("mpu9250: irq unknown code.\n");
            
            break;
        }
    }
	 }
void a_dmp_tap_callback(uint8_t count, uint8_t direction)
	 {
		dmp_tap_Count++;
	 }
void a_dmp_orient_callback(uint8_t orientation)
	 {
		dmp_orient_Count++;
	 }




//imu_update.h
// 四元数与向量相乘，旋转向量
Vector3 quat_rotate(Quaternion q, Vector3 v) {
    Vector3 result;
    
    double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    double vx = v.x, vy = v.y, vz = v.z;
    
    // 计算四元数与向量的乘积
    result.x = qw*qw*vx + 2*qy*qw*vz - 2*qz*qw*vy + qx*qx*vx + 2*qy*qx*vy + 2*qz*qx*vz - qz*qz*vx - qy*qy*vx;
    result.y = 2*qx*qy*vx + qy*qy*vy + 2*qz*qy*vz + 2*qw*qz*vx - qz*qz*vy + qw*qw*vy - 2*qx*qw*vz - qx*qx*vy;
    result.z = 2*qx*qz*vx + 2*qy*qz*vy + qz*qz*vz - 2*qw*qy*vx - qy*qy*vz + 2*qw*qx*vy - qx*qx*vz + qw*qw*vz;

    return result;
}

// 向量加法
Vector3 vec_add(Vector3 a, Vector3 b) {
    Vector3 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

// 向量减法
Vector3 vec_sub(Vector3 a, Vector3 b) {
    Vector3 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

// 向量标量乘法
Vector3 vec_scale(Vector3 v, double s) {
    Vector3 result;
    result.x = v.x * s;
    result.y = v.y * s;
    result.z = v.z * s;
    return result;
}

// 位置更新函数
// 输入：q姿态四元数 a_b本体坐标系下的加速度 v速度 p位置 g三维重力加速度 deltaT采样时间
// 输出：更新的p位置



void update_position(Quaternion q, Vector3 a_b, Vector3* v, Vector3* p, Vector3 g, double deltaT) {
    // 将加速度转换到全球坐标系
    Vector3 a_g = quat_rotate(q, a_b);
    app_log_info("加速度(转换后) X:%f,Y:%f,Z:%f\n",a_g.x,a_g.y,a_g.z);


    // 分离重力加速度
    Vector3 a_d = vec_sub(a_g, g);

    // 更新速度
    *v = vec_add(*v, vec_scale(a_d, deltaT));

    // 更新位置
    *p = vec_add(*p, vec_add(vec_scale(*v, deltaT), vec_scale(a_d, 0.5 * deltaT * deltaT)));
}

//输入q  a_b g  输出分离重力加速度的 加速度
void update_acc(Quaternion q, Vector3 a_b,Vector3 g, Vector3* acc) 
{
    // 将加速度转换到全球坐标系
    Vector3 a_g = quat_rotate(q, a_b);
    app_log_info("加速度(转换后) X:%f,Y:%f,Z:%f\n",a_g.x,a_g.y,a_g.z);
    // 分离重力加速度
    Vector3 a_d = vec_sub(a_g, g);
    *acc=a_d;
    app_log_info("加速度(转换后) X:%f,Y:%f,Z:%f\n",a_d.x,a_d.y,a_d.z);
}

