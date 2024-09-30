#include <stdio.h>
#include "driver/adc.h"

typedef struct
{
    adc1_channel_t channel;
    adc_atten_t atten;
    
} lm35_object_t;

void lm35_init(adc1_channel_t channel, adc_atten_t atten);

void lm35_init_v2(lm35_object_t sensor);
