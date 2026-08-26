/**
 * @file main.c
 * @author cappy
 * @date 2026-08-21
 * @brief Main function
 */

#include <xc.h>
#include "clock.h"
#include "led.h"
#include "uart.h"
#define _XTAL_FREQ 64000000UL

int main(){
    clk_initialize();
    setup_led();
    UART_Init();
    const char *message = "Hello, UART!\n";
    // Add your code here and press Ctrl + Shift + B to build
    while(1) {
        led_toggle();
        uart_send_string(message);
        __delay_ms(500);
    }

    return 0;
}
