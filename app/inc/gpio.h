#ifndef GPIO_H_
#define GPIO_H_

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/stm32/exti.h"

#include "core/uart.h"
#include "lynxmotion.h"

#define BUILTIN_BU_PORT     (GPIOC)
#define BUILTIN_BU_PIN      (GPIO13)


void setup_push_button(void);

#endif