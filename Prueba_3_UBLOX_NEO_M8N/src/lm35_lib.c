
#include "lm35_lib.h"


void lm35_init(adc1_channel_t channel, adc_atten_t atten)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);
}


void lm35_init_v2(lm35_object_t sensor)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(sensor.channel, sensor.atten);
}

double lm35_read_temp(adc1_channel_t channel)
{
    adc1_get_raw(channel);
}


