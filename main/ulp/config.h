#pragma once

/* Ints are used here to be able to include the file in assembly as well */
#define ADC_CHANNEL     7 // ADC_CHANNEL_7
#define ADC_UNIT        0 // ADC_UNIT_1
#define ADC_ATTEN       3 // ADC_ATTEN_DB_12
#define ADC_WIDTH       0 // ADC_BITWIDTH_DEFAULT

/* Set low and high thresholds, approx. 1.35V - 1.75V*/
#define ADC_LOW_TRESHOLD    0 // 1500
#define ADC_HIGH_TRESHOLD   512 // 2000
