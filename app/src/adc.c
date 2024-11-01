#include "adc.h"

void loc_adc_setup(void)
{
    
    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_sample_time(ADC1, ADC_CHANNEL4, ADC_SMPR_SMP_3CYC);
    adc_power_on(ADC1);
}

void loc_adc_setup_advanced(void)
{
    
}

uint16_t loc_read_adc(uint8_t channel)
{
    uint8_t channel_array[16];
    channel_array[0] = channel;
    adc_set_regular_sequence(ADC1, 1, channel_array);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1));
    uint16_t reg16 = adc_read_regular(ADC1);
    return reg16;
}