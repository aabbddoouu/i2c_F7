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
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include "ssd1306.h"
//#include <../SSD1306/ssd1306/ssd1306.h>

#define AUTOEND true
#define I2C_OLED I2C1

void i2c_xfer7(uint32_t i2c, uint8_t addr, uint8_t command, uint8_t *w, size_t wn, bool);
void delay_ms(uint32_t ms);

static int uart_printf(const char *format,...) __attribute((format(printf,1,2)));