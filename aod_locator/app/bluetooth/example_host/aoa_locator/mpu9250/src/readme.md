怎么使用？
    Quaternion q = {0.7071, 0.7071, 0, 0}; // 示例四元数
    Vector3 a_b = {0, 0, 9.81}; // 本体坐标系下的加速度
    Vector3 v = {0, 0, 0}; // 初始速度
    Vector3 p = {0, 0, 0}; // 初始位置
    Vector3 g = {0, 0, -9.81}; // 重力加速度（全球坐标系）
    double deltaT = 0.01; // 时间间隔

    // 更新位置
    update_position(q, a_b, &v, &p, g, deltaT);

    // 输出结果
    printf("位置: x = %f, y = %f, z = %f\n", p.x, p.y, p.z);
    printf("速度: x = %f, y = %f, z = %f\n", v.x, v.y, v.z);


    修改了print/
    
2024.5.20
指定了交叉编译器 并指定了arm架构
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake ..
主要内容：
set(CMAKE_C_COMPILER /home/c606/jieryyyyy/gcc/gcc-linaro-7.5.0-2019.12-i686_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_SYSTEM_PROCESSOR arm)


加入DMP



加入中断和gpiod库(已被删除，因为正点原子的Linux内核不支持gpiod，使用的sys)
添加了gpiod.h 以及libgpiod.a


5.27 
将        if (gpio_interrupt_init() != 0)
        {
            mpu9250_interface_debug_print("gpio_interrupt_init failed\n");
            return 1;
        }
        g_gpio_irq = mpu9250_dmp_irq_handler;

放到了dmp初始化前面





