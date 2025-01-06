#ifndef GPIO_H_
#define GPIO_H_

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/stm32/exti.h"

#include "core/uart.h"
#include "lynxmotion.h"
#include "fault-handler.h"

#define BUILTIN_BU_PORT     (GPIOC)
#define BUILTIN_BU_PIN      (GPIO13)

#define BUILTIN_LD2_PORT    (GPIOA)
#define BUILTIN_LD2_PIN     (GPIO5)

#define UART_PORT           (GPIOA)
#define UART_TX_PIN         (GPIO2)
#define UART_RX_PIN         (GPIO3)

#define SERVO_A_PORT            (GPIOA)
#define SERVO_BASE_PIN          (GPIO0)
#define SERVO_BASE_UPPER_PIN    (GPIO1)


#define SERVO_B_PORT            (GPIOB)
#define SERVO_GRIPPER_PIN       (GPIO4)
#define SERVO_ELBOW_PIN         (GPIO6)
#define SERVO_WRIST_LOWER_PIN   (GPIO10)

#define SERVO_C_PORT            (GPIOC)
#define SERVO_WRIST_UPPER_PIN   (GPIO7)

#define ADC_PORT                (GPIOA)
#define ADC_PIN                 (GPIO4)


void loc_gpio_setup(void);
void setup_push_button(void);
void setup_emergency_stop_button(void);
void toggle_status_led(char colour);

#endif