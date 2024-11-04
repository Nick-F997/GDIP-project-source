#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdlib.h>

#include "libopencm3/stm32/adc.h"
#include "libopencm3/stm32/rcc.h"

#define ADC_CHANNEL_NUM (6)
#define ADC_LOCAL_SAMPLE_RATE (15)

typedef struct ADCControl_t
{
    uint8_t channel;
    uint16_t current_reading;
    uint8_t sample_depth;
} ADCControl_t;

void loc_adc_setup(void);
uint16_t loc_read_adc(uint8_t channel);
uint16_t *read_all_channels(void);
uint16_t read_channel_averaged(uint8_t channel, uint16_t sample_depth);
ADCControl_t get_adc_channel(uint16_t joint);



#endif