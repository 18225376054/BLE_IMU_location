#include "gpio.h"


#define GPIO_CHIP_PATH "/sys/class/gpio/gpiochip0"
#define GPIO_PIN_NUM "1"
#define GPIO_PIN_VALUE "/sys/class/gpio/gpio1/value"
#define GPIO_PIN_DIRECTION "/sys/class/gpio/gpio1/direction"
#define GPIO_PIN_EDGE "/sys/class/gpio/gpio1/edge"
#define MAX_BUF 64

// 声明函数指针
extern uint8_t (*g_gpio_irq)(void);

static pthread_t gpio_thread;

static void *gpio_interrupt_thread(void *arg) {
    int fd_value;
    char value;
    while (1) {
        // Read GPIO pin value
        fd_value = open(GPIO_PIN_VALUE, O_RDONLY);
        if (fd_value < 0) {
            perror("gpio: open value failed");
            exit(1);
        }
        if (read(fd_value, &value, 1) != 1) {
            perror("gpio: read value failed");
            exit(1);
        }
        close(fd_value);
        
        // Run the callback when falling edge detected
        if (value == '0') {
            // Call the callback function
            // Replace the following line with your actual callback function
            //这里应该添加真正的中断处理函数  添加代码 mutex_irq
            printf("Falling edge detected\n");
            mutex_irq(g_gpio_irq);
        }
        
        // Sleep for a while to avoid busy loop
        usleep(100000); // 100ms
    }
    return NULL;
}

// uint8_t gpio_interrupt_init(void) {
//     int fd_direction, fd_edge;
//     pthread_attr_t attr;
//     char buf[MAX_BUF];
//     int res;
//     printf("中断初始化0\n");
//     // Export GPIO pin
// //     int fd_unexport  = open("/sys/class/gpio/export", O_WRONLY);
// //     // if (fd_export < 0) {
// //     //     perror("gpio: export failed");
// //     //     return 1;
// //     // }
// // if (fd_unexport < 0) {
// //     perror("gpio: unexport failed");
// //     // It's okay if unexport fails, maybe the pin is not yet exported
// // } else {
// //     // Unexport GPIO pin
// //     if (write(fd_unexport, "GPIO_PIN_NUM", strlen("GPIO_PIN_NUM")) < 0) {
// //         perror("gpio: unexport write failed");
// //         close(fd_unexport);
// //         return 1;
// //     }
// //     close(fd_unexport);
// // }

// // printf("中断初始化1\n");
// // Export GPIO pin
// int fd_export = open("/sys/class/gpio/export", O_WRONLY);
// if (fd_export < 0) {
//     perror("gpio: export failed");
//     return 1;
// }




//     if (write(fd_export, GPIO_PIN_NUM, sizeof(GPIO_PIN_NUM)) < 0) {
//         perror("gpio: write export failed");
//         close(fd_export);
//         return 1;
//     }
//     close(fd_export);
    
//     // Set GPIO pin direction to input
//     fd_direction = open(GPIO_PIN_DIRECTION, O_WRONLY);
//     if (fd_direction < 0) {
//         perror("gpio: set direction failed");
//         return 1;
//     }
//     if (write(fd_direction, "in", 2) < 0) {
//         perror("gpio: write direction failed");
//         close(fd_direction);
//         return 1;
//     }
//     close(fd_direction);
    
//     // Set GPIO pin edge to falling
//     fd_edge = open(GPIO_PIN_EDGE, O_WRONLY);
//     if (fd_edge < 0) {
//         perror("gpio: set edge failed");
//         return 1;
//     }
//     if (write(fd_edge, "falling", 7) < 0) {
//         perror("gpio: write edge failed");
//         close(fd_edge);
//         return 1;
//     }
//     close(fd_edge);
    
//     // Create a GPIO interrupt thread
//     pthread_attr_init(&attr);
//     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
//     res = pthread_create(&gpio_thread, &attr, gpio_interrupt_thread, NULL);
//     if (res != 0) {
//         perror("gpio: create thread failed");
//         return 1;
//     }
//     printf("创建线程\n");
//     pthread_attr_destroy(&attr);
    
//     return 0;
// }



//软件打开后  需要在开发板上使用
// echo "1" > /sys/class/gpio/unexport
// 关闭GPIO口
uint8_t gpio_interrupt_init(void) {
    int fd_export, fd_unexport, fd_direction, fd_edge;
    pthread_attr_t attr;
    char buf[MAX_BUF];
    int res;

    printf("中断初始化0\n");

    // 尝试导出 GPIO 引脚
    fd_export = open("/sys/class/gpio/export", O_WRONLY);
    if (fd_export < 0) {
        perror("gpio: export open failed");
        
        // 尝试取消导出 GPIO 引脚
        fd_unexport = open("/sys/class/gpio/unexport", O_WRONLY);
        if (fd_unexport < 0) {
            perror("gpio: unexport open failed");
            return 1;
        }
        if (write(fd_unexport, GPIO_PIN_NUM, strlen(GPIO_PIN_NUM)) < 0) {
            perror("gpio: unexport write failed");
            close(fd_unexport);
            return 1;
        }
        close(fd_unexport);
        

    }
        // 重新尝试导出 GPIO 引脚
        fd_export = open("/sys/class/gpio/export", O_WRONLY);
        if (fd_export < 0) {
            perror("gpio: export open failed again");
            return 1;
        }

    if (write(fd_export, GPIO_PIN_NUM, strlen(GPIO_PIN_NUM)) < 0) {
        perror("gpio: write export failed");
        close(fd_export);
        return 1;
    }
    close(fd_export);

    printf("中断初始化1\n");

    // 设置 GPIO 引脚方向为输入
    fd_direction = open(GPIO_PIN_DIRECTION, O_WRONLY);
    if (fd_direction < 0) {
        perror("gpio: set direction failed");
        return 1;
    }
    if (write(fd_direction, "in", 2) < 0) {
        perror("gpio: write direction failed");
        close(fd_direction);
        return 1;
    }
    close(fd_direction);

    // 设置 GPIO 引脚中断边沿为 falling
    fd_edge = open(GPIO_PIN_EDGE, O_WRONLY);
    if (fd_edge < 0) {
        perror("gpio: set edge failed");
        return 1;
    }
    if (write(fd_edge, "falling", 7) < 0) {
        perror("gpio: write edge failed");
        close(fd_edge);
        return 1;
    }
    close(fd_edge);

    // 创建 GPIO 中断线程
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    res = pthread_create(&gpio_thread, &attr, gpio_interrupt_thread, NULL);
    if (res != 0) {
        perror("gpio: create thread failed");
        return 1;
    }
    printf("创建线程\n");
    pthread_attr_destroy(&attr);

    return 0;
}







uint8_t gpio_interrupt_deinit(void) {
    int fd_unexport;
    
    // Cancel GPIO interrupt thread
    pthread_cancel(gpio_thread);
    
    // Unexport GPIO pin
    fd_unexport = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd_unexport < 0) {
        perror("gpio: unexport failed");
        return 1;
    }
    if (write(fd_unexport, GPIO_PIN_NUM, sizeof(GPIO_PIN_NUM)) < 0) {
        perror("gpio: write unexport failed");
        close(fd_unexport);
        return 1;
    }
    close(fd_unexport);
    
    return 0;
}
