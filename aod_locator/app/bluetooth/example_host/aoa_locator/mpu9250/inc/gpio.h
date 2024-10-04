#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include "mutex.h"


static void *gpio_interrupt_thread(void *arg);

uint8_t gpio_interrupt_init(void);
uint8_t gpio_interrupt_deinit(void);
