#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/cm3/scb.h" // For vector table offset register

#include "lynxmotion.h"

#include "core/system.h"
#include "core/uart.h"
#include "gpio.h"
#include "sys_timer.h"
#include "adc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Define Robot state here


#define BOOTLOADER_SIZE     (0x8000U)



static void loc_vector_setup(void) {
    SCB_VTOR = BOOTLOADER_SIZE; // Offset main Vector Table by size of bootloader so it knows where to look.
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
    setRobotState(STATE_INIT);
    coreSystemSetup();
    loc_gpio_setup();
    setup_push_button();
    coreUartSetup(115200);
    loc_adc_setup();

    setRobotState(STATE_TEACH_POS1);

    coreUartWrite("Initialisation Completed!\r\n", 28);
    uint64_t time_in_move = coreGetTicks();

    State previous_state = getRobotState();
    while (1)
    {
        State current_state = getRobotState();
        switch (current_state)
        {
            case STATE_TEACH_POS1:
            {
                if (previous_state != current_state)
                {
                    coreUartWrite("I am in teach mode 1\r\n", 23);
                }
                uint16_t adc_val = loc_read_adc(4);

                
                
                char buffer[128];
                sprintf(buffer, "Pot Val - %d\r\n", adc_val);
                coreUartWrite(buffer, strlen(buffer));
                break;
            }
            case STATE_TEACH_POS2:
            {
                if (previous_state != current_state)
                {
                    coreUartWrite("I am in teach mode 2\r\n", 23);
                }
                
                uint16_t *adc_vals = read_all_channels();

                
                char buffer[128];
                sprintf(buffer, "pot1 = %d\tpot2 = %d\tpot3 = %d\tpot4 = %d\tpot5 = %d\tpot6 = %d\r\n", adc_vals[0], adc_vals[1], adc_vals[2], adc_vals[3], adc_vals[4], adc_vals[5]);
                coreUartWrite(buffer, strlen(buffer));
                free(adc_vals);
                break;
            }
            case STATE_MOVING_POS1:
            {
                if (previous_state != current_state)
                {
                    time_in_move = coreGetTicks();
                    coreUartWrite("I am in move mode 1\r\n", 22);
                }
                if (coreGetTicks() - time_in_move > 5000)
                {
                    setRobotState(STATE_MOVING_POS2);
                }
                break;
            }
            case STATE_MOVING_POS2:
            {
                if (previous_state != current_state)
                {
                    time_in_move = coreGetTicks();
                    coreUartWrite("I am in move mode 2\r\n", 22);
                }
                
                if (coreGetTicks() - time_in_move > 5000)
                {
                    setRobotState(STATE_MOVING_POS1);
                }
                break;
            }
            default:
            {
                coreUartWrite("You shouldn't be here!\r\n", 25);
                break;
            }
        }
        previous_state = current_state;
        coreSystemDelay(100);
    }

    return 0;
}

// int main(void)
// {
//     loc_vector_setup();
//     coreSystemSetup();
//     loc_gpio_setup();
//     //coreTimerSetup();
//     // TimerControl_t timer2;
//     // TimerControl_t timer3;
//     // TimerControl_t timer4;
//     // coreTimerSetupReturns(&timer2, RCC_TIM2, TIM2, 84 + 1, 20000 + 1, TIM_OC1);

//     LynxMotion_t arm;
//     MotorSetup(&arm);
//     coreUartSetup(115200);

//     uint64_t start_time = coreGetTicks();
//     float duty_cycle = 7.9f;

//     // corePWMSetDutyCycleStruct(&timer2, duty_cycle);
//     setPWMforRobotWithDelay(&arm, duty_cycle, 1000);

//     while (1)
//     {
//         while (coreUartDataAvailable()) 
//         {

//             // coreUartWriteByte(data);
//             uint8_t data = coreUartReadByte();

//             switch (data) {
//                 case '+': {
//                     duty_cycle += 0.1f;
//                     break;
//                 }
//                 case '-': {
//                     duty_cycle -= 0.1f;
//                 }
//                 default:
//                 {
//                     break;
//                 }
//             }
            
//         }
        
//         if (duty_cycle > 13.2f)
//         {
//             duty_cycle = 13.2f;
//         }

//         if (duty_cycle < 3.3f)
//         {
//             duty_cycle = 3.3f;
//         }
//         //corePWMSetDutyCycleStruct(&timer2, duty_cycle);
//         setPWMforRobotWithDelay(&arm, duty_cycle, 500);
//         char buffer[128];
//         sprintf(buffer, "duty cycle - %.1f%%\r\n", duty_cycle);
//         coreUartWrite(buffer, strlen(buffer));
        
        
//         coreSystemDelay(10);
//     }
//     // Should never get here
//     return 0;
// }