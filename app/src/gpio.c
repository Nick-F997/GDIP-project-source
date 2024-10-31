#include "gpio.h"

#include <stdio.h>
#include <string.h>

void setup_push_button(void)
{
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(BUILTIN_BU_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, BUILTIN_BU_PIN);

    // setup exti
    rcc_periph_clock_enable(RCC_SYSCFG);

    nvic_enable_irq(NVIC_EXTI15_10_IRQ);
    exti_select_source(EXTI13, BUILTIN_BU_PORT);
    exti_set_trigger(EXTI13, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI13);
}

void exti15_10_isr(void)
{
    if (exti_get_flag_status(EXTI13))
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
        exti_reset_request(EXTI13);
        
    }
}

