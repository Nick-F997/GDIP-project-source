#include "adc.h"

static volatile uint8_t channels[ADC_CHANNEL_NUM] = {4, 8, 11, 10, 6, 7};

static uint16_t sum_avg_uint16t(uint16_t *arr, int size)
{
    uint16_t sum = 0;
    for (int i = 0; i < size; i++)
    {
        sum += arr[i];
    }

    return (uint16_t)(sum / size);
}



void loc_adc_setup(void)
{
    
    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_set_sample_time(ADC1, ADC_CHANNEL4, ADC_SMPR_SMP_3CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL6, ADC_SMPR_SMP_3CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL7, ADC_SMPR_SMP_3CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL8, ADC_SMPR_SMP_3CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL10, ADC_SMPR_SMP_3CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL11, ADC_SMPR_SMP_3CYC);
    adc_power_on(ADC1);
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

uint16_t read_channel_averaged(uint8_t channel, uint16_t sample_depth)
{
    uint16_t avgVals = 0;
    for (int _i = 0; _i < sample_depth; _i++)
    {
        uint16_t value = loc_read_adc(channel);
        avgVals += value;
    }

    (uint16_t)(avgVals / sample_depth);

}

uint16_t *read_all_channels(void)
{
    uint16_t *channelValues = (uint16_t *)malloc(sizeof(uint16_t) * ADC_CHANNEL_NUM);
    for (int channel = 0; channel < ADC_CHANNEL_NUM; channel++)
    {
        uint16_t avgVals[ADC_LOCAL_SAMPLE_RATE];
        for (int sample = 0; sample < ADC_LOCAL_SAMPLE_RATE; sample++)
        {
            avgVals[sample] = loc_read_adc(channels[channel]);

        }
        
        channelValues[channel] = sum_avg_uint16t(avgVals, ADC_LOCAL_SAMPLE_RATE);
    }

    return channelValues;
}

ADCControl_t get_adc_channel(uint16_t joint)
{
    return (ADCControl_t) {.channel = channels[joint], .current_reading = 0, .sample_depth = ADC_LOCAL_SAMPLE_RATE};
}