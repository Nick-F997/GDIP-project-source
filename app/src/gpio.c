#include "gpio.h"

#include <stdio.h>
#include <string.h>

static void adc_gpio_setup(void)
{
    // Port A
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO6);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO7);

    // Port B
    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    
    // Port C
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
}


void loc_gpio_setup(void)
{
    // Enable GPIO rcc clock
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    
    gpio_mode_setup(BUILTIN_LD2_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BUILTIN_LD2_PIN | SERVO_BASE_PIN | SERVO_BASE_UPPER_PIN); // Set LED2 pin to Alternative Function mode
    gpio_set_af(BUILTIN_LD2_PORT, GPIO_AF1, BUILTIN_LD2_PIN | SERVO_BASE_PIN | SERVO_BASE_UPPER_PIN);
    
    // GPIO setup for uart. Clock for peripheral is set elsewhere
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, UART_TX_PIN | UART_RX_PIN /* boolean or in order to set both pins*/);
    gpio_set_af(UART_PORT, GPIO_AF7, UART_TX_PIN | UART_RX_PIN); // See datasheet for function.

    gpio_mode_setup(SERVO_B_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SERVO_GRIPPER_PIN | SERVO_ELBOW_PIN | SERVO_WRIST_LOWER_PIN);
    gpio_set_af(SERVO_B_PORT, GPIO_AF2, SERVO_GRIPPER_PIN | SERVO_ELBOW_PIN);
    gpio_set_af(SERVO_B_PORT, GPIO_AF1, SERVO_WRIST_LOWER_PIN);

    gpio_mode_setup(SERVO_C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SERVO_WRIST_UPPER_PIN);
    gpio_set_af(SERVO_C_PORT, GPIO_AF2, SERVO_WRIST_UPPER_PIN);
    
    // Setup button 
    // gpio_mode_setup(BUILTIN_BU_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BUILTIN_BU_PIN);
    adc_gpio_setup();

}


void setup_push_button(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO10);

    // setup exti
    rcc_periph_clock_enable(RCC_SYSCFG);

    nvic_enable_irq(NVIC_EXTI15_10_IRQ);
    exti_select_source(EXTI10, GPIOA);
    exti_set_trigger(EXTI10, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI10);
}

void exti15_10_isr(void)
{
    if (exti_get_flag_status(EXTI10))
    {
        switch (getRobotState())
        {
            case STATE_INIT:
            case STATE_EMERGENCY_STOP:
            {
                break;
            }
            case STATE_TEACH_POS1:
            {
                setRobotState(STATE_TEACH_POS2);
                break;
            }
            case STATE_TEACH_POS2:
            {
                setRobotState(STATE_MOVING_POS1);
                break;
            }
            case STATE_MOVING_POS1:
            case STATE_MOVING_POS2:
            {
                setRobotState(STATE_TEACH_POS1);
                break;
            }
        }
        exti_reset_request(EXTI10);
        
    }
}

