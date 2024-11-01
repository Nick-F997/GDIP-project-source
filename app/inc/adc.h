#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

#include "libopencm3/stm32/adc.h"
#include "libopencm3/stm32/rcc.h"

void loc_adc_setup(void);
uint16_t loc_read_adc(uint8_t channel);

#endif