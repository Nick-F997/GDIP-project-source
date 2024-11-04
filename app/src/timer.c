#include "sys_timer.h"

//#include "libopencm3/stm32/rcc.h"

#define PRESCALER (84)
#define ARR_VALUE (1000)

/**
 * @brief Sets up timer for PWM. Initially at 84_000_000 Hz. 
 * 
 * @param prescaler how much to scale the inital frequency at. Recommended 84. Actual 
 * calculation: sysfreq / ((prescaler - 1) * (arr_val - 1))
 * @param arr_val the value at which the register reloads (Auto Reload Register). Recommended 1000
 * @return a pointer to a struct that contains information about the PWM peripheral
 */
void coreTimerSetup(void)
{
    // Enable clock to TIM2
    rcc_periph_clock_enable(RCC_TIM2);
    // set mode:   timer2, no clock div,     edge aligned,     count up
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    // set mode to pwm on channel 1
    timer_set_oc_mode(TIM2, TIM_OC1 /* channel 1 */, TIM_OCM_PWM1);
    // Setup PWM output compare and enable counter
    timer_enable_counter(TIM2);
    timer_enable_oc_output(TIM2, TIM_OC1);

    // Set the prescaler and auto reload register (resolution and frequency)
    timer_set_prescaler(TIM2, PRESCALER - 1);
    timer_set_period(TIM2, ARR_VALUE - 1);
}


/**
 * @brief Coverts a floating point number into a CCR value for PWM.
 * 
 * @param duty_cycle 
 */
void corePWMSetDutyCycle(float duty_cycle)
{
    const float raw_val = (float)ARR_VALUE * (duty_cycle / 100.0f);
    timer_set_oc_value(TIM2, TIM_OC1, (uint32_t)raw_val);
}



void corePWMSetDutyCycleStruct(TimerControl_t *controller, float duty_cycle)
{
    controller->duty_cycle = duty_cycle;
    controller->pulse_width = (int)(((controller->duty_cycle) / 100.0f) * controller->period);
    const float raw_val = (float)controller->arr_value * (controller->duty_cycle / 100.0f);
    timer_set_oc_value(controller->timer, controller->channel, (uint32_t)raw_val);
}

void coreTimerSetupReturns(TimerControl_t *controller, enum rcc_periph_clken periph_clock, uint32_t timer, 
                            int prescaler, int arr_value, enum tim_oc_id channel)
{
    
    controller->timer = timer;
    controller->channel = channel;
    controller->prescaler = prescaler - 1;
    controller->arr_value = arr_value - 1;
    controller->duty_cycle = 0;
    controller->pulse_width = 0;
    controller->frequency = (int)(84000000 / ((prescaler) * (arr_value)));
    controller->period = (int)((1.0f / (float)controller->frequency) * 1000000.0f);

    // Enable clock to TIM2
    // rcc_periph_clock_enable(periph_clock); removing for now as I don't know what happens if you call it twice
    // set mode:   timer2, no clock div,     edge aligned,     count up
    timer_set_mode(controller->timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    
    timer_set_oc_mode(controller->timer, controller->channel, TIM_OCM_PWM1);

    // Setup PWM output compare and enable counter
    timer_enable_counter(controller->timer);
    timer_enable_oc_output(controller->timer, controller->channel);

    // Set the prescaler and auto reload register (resolution and frequency)
    timer_set_prescaler(controller->timer, controller->prescaler - 1);
    timer_set_period(controller->timer, controller->arr_value - 1);
}