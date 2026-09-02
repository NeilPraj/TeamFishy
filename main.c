/**
 * @file main.c
 * @author cappy
 * @date 2026-08-21
 * @brief Main function
 */

#include "adc.h"
#include "clock.h"
#include "led.h"
#include "uart.h"
#include <xc.h>
#define _XTAL_FREQ 64000000UL

int main() {
  clk_initialize();
  setup_led();
  UART_Init();
  ADC_init();
  const char *message = "Hello, UART!\n";
  uart_send_string(message);

  while (1) {
    led_toggle();
    uint16_t res = ADC_read(ADC_CHANNEL_POT);
    __delay_ms(500);
  }

  return 0;
}
