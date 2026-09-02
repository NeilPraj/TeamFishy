#include "adc.h"
#include "board.h"
#include <xc.h>

void ADC_init(void) {
  ADC_TRIS = 1;
  ADC_ANSEL = 1;
  ADC_CHANNEL_SEL = ADC_CHANNEL_1;
  ADC_CLOCK = ADC_CLOCK_DIV;
  ADC_ACQ = ADC_ACQ_TIME;
  ADC_REF_REG.ADPREF = ADC_PREF;
  ADC_REF_REG.ADNREF = ADC_NREF;
  ADC_FORMAT = ADC_FORMAT_RIGHT;
  ADC_ON = 1;
}

void ADC_startConversion(adc_channel_t channel) {
  ADC_CHANNEL_SEL = channel;
  ADC_GO = 1;
}

uint8_t ADC_isConversionDone(void) { return (!ADC_GO); }

uint16_t ADC_getResult(void) {
  uint16_t res = ((uint16_t)ADC_RESULT_H << 8 | ADC_RESULT_L);
  return res;
}

uint16_t ADC_read(adc_channel_t channel) {
  ADC_startConversion(channel);
  while (!ADC_isConversionDone()) {
    __nop();
  }
  return ADC_getResult();
}
