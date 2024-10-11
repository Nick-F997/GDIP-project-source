#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/cm3/scb.h" // For vector table offset register

#include "core/system.h"
#include "core/uart.h"
#include "sys_timer.h"

#include <stdio.h>
#include <string.h>

#define BOOTLOADER_SIZE     (0x8000U)

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


static void loc_vector_setup(void) {
    SCB_VTOR = BOOTLOADER_SIZE; // Offset main Vector Table by size of bootloader so it knows where to look.
}

static void loc_gpio_setup(void)
{
    // Enable GPIO rcc clock
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(BUILTIN_LD2_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BUILTIN_LD2_PIN | SERVO_BASE_PIN | SERVO_BASE_UPPER_PIN); // Set LED2 pin to Alternative Function mode
    gpio_set_af(BUILTIN_LD2_PORT, GPIO_AF1, BUILTIN_LD2_PIN | SERVO_BASE_PIN | SERVO_BASE_UPPER_PIN);

    // GPIO setup for uart. Clock for peripheral is set elsewhere
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, UART_TX_PIN | UART_RX_PIN /* boolean or in order to set both pins*/);
    gpio_set_af(UART_PORT, GPIO_AF7, UART_TX_PIN | UART_RX_PIN); // See datasheet for function.

    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(SERVO_B_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SERVO_GRIPPER_PIN | SERVO_ELBOW_PIN | SERVO_WRIST_LOWER_PIN);
    gpio_set_af(SERVO_B_PORT, GPIO_AF2, SERVO_GRIPPER_PIN | SERVO_ELBOW_PIN);
    gpio_set_af(SERVO_B_PORT, GPIO_AF1, SERVO_WRIST_LOWER_PIN);

    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(SERVO_C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SERVO_WRIST_UPPER_PIN);
    gpio_set_af(SERVO_C_PORT, GPIO_AF2, SERVO_WRIST_UPPER_PIN);
}

static void MotorSetup(LynxMotion_t *arm)
{
    rcc_periph_clock_enable(RCC_TIM2);
    coreTimerSetupReturns(&arm->baseJoint, RCC_TIM2, TIM2, 84 + 1, 20000 + 1, TIM_OC1);
    coreTimerSetupReturns(&arm->baseUpperJoint, RCC_TIM2, TIM2, 84 + 1, 20000 + 1, TIM_OC2);
    coreTimerSetupReturns(&arm->wristLowerJoint, RCC_TIM2, TIM2, 84 + 1, 20000 + 1, TIM_OC3);
    
    rcc_periph_clock_enable(RCC_TIM3);
    coreTimerSetupReturns(&arm->gripperJoint, RCC_TIM3, TIM3, 84 + 1, 20000 + 1, TIM_OC1);
    coreTimerSetupReturns(&arm->wristUpperJoint, RCC_TIM3, TIM3, 84 + 1, 20000 + 1, TIM_OC2);
    
    rcc_periph_clock_enable(RCC_TIM4);
    coreTimerSetupReturns(&arm->elbowJoint, RCC_TIM4, TIM4, 84 + 1, 20000 + 1, TIM_OC1);
}


static void setPWMforRobotWithDelay(LynxMotion_t *arm, float duty_cycle, uint64_t delay)
{
    corePWMSetDutyCycleStruct(&arm->baseJoint, duty_cycle);
    coreSystemDelay(delay);

    corePWMSetDutyCycleStruct(&arm->baseUpperJoint, duty_cycle);
    coreSystemDelay(delay);

    corePWMSetDutyCycleStruct(&arm->elbowJoint, duty_cycle);
    coreSystemDelay(delay);

    corePWMSetDutyCycleStruct(&arm->wristUpperJoint, duty_cycle);
    coreSystemDelay(delay);

    corePWMSetDutyCycleStruct(&arm->wristLowerJoint, duty_cycle);
    coreSystemDelay(delay);

    corePWMSetDutyCycleStruct(&arm->gripperJoint, duty_cycle);
}


int main(void)
{
    loc_vector_setup();
    coreSystemSetup();
    loc_gpio_setup();
    //coreTimerSetup();
    // TimerControl_t timer2;
    // TimerControl_t timer3;
    // TimerControl_t timer4;
    // coreTimerSetupReturns(&timer2, RCC_TIM2, TIM2, 84 + 1, 20000 + 1, TIM_OC1);

    LynxMotion_t arm;
    MotorSetup(&arm);
    coreUartSetup(115200);

    uint64_t start_time = coreGetTicks();
    float duty_cycle = 7.9f;

    // corePWMSetDutyCycleStruct(&timer2, duty_cycle);
    setPWMforRobotWithDelay(&arm, duty_cycle, 1000);

    while (1)
    {
        while (coreUartDataAvailable()) 
        {

            // coreUartWriteByte(data);
            uint8_t data = coreUartReadByte();

            switch (data) {
                case '+': {
                    duty_cycle += 0.1f;
                    break;
                }
                case '-': {
                    duty_cycle -= 0.1f;
                }
                default:
                {
                    break;
                }
            }
            
        }
        
        if (duty_cycle > 13.2f)
        {
            duty_cycle = 13.2f;
        }

        if (duty_cycle < 3.3f)
        {
            duty_cycle = 3.3f;
        }
        //corePWMSetDutyCycleStruct(&timer2, duty_cycle);
        setPWMforRobotWithDelay(&arm, duty_cycle, 500);
        char buffer[128];
        sprintf(buffer, "duty cycle - %.1f%%\r\n", duty_cycle);
        coreUartWrite(buffer, strlen(buffer));
        
        
        coreSystemDelay(10);
    }
    // Should never get here
    return 0;
}