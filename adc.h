#include "board.h"
#include <xc.h>

typedef enum {
  ADC_CHANNEL_POT,
} adc_channel_t;

void ADC_init(void);

void ADC_startConversion(adc_channel_t channel);

uint8_t ADC_isConversionDone(void);

uint16_t ADC_getResult(void);

uint16_t ADC_read(adc_channel_t channel);
