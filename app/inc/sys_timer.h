#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>
#include <stdlib.h>

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/timer.h"

typedef struct {
    uint32_t timer;
    enum tim_oc_id channel;
    uint32_t prescaler;
    uint32_t arr_val;
    float duty_cycle;
} PWMPeripheral;

void coreTimerSetup(void);
void corePWMSetDutyCycle(float duty_cycle);


typedef struct TimerControl_t {
    uint32_t timer;
    enum tim_oc_id channel;
    float duty_cycle; // %
    int pulse_width; // us as they're easier to see.
    int frequency; // Hz
    int period; // us as they're easier to see
    uint32_t arr_value;
    uint32_t prescaler;
} TimerControl_t;

typedef struct LynxMotion_t {
    TimerControl_t baseJoint;
    TimerControl_t baseUpperJoint;
    TimerControl_t wristLowerJoint;
    TimerControl_t gripperJoint;
    TimerControl_t wristUpperJoint;
    TimerControl_t elbowJoint;
} LynxMotion_t;

void coreTimerSetupReturns(TimerControl_t *controller, enum rcc_periph_clken periph_clock, uint32_t timer, int prescaler, int arr_value, enum tim_oc_id channel);
void corePWMSetDutyCycleStruct(TimerControl_t *controller, float duty_cycle);
#endif