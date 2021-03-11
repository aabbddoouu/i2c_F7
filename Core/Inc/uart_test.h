#ifndef STM32F7
#define STM32F7
#endif

#include "miniprintf.h"
#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <lsm6ds3_STdC/driver/lsm6ds3_reg.h>

static int uart_printf(const char *format,...) __attribute((format(printf,1,2)));
