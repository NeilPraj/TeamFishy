#include <xc.h>
#include "led.h"
#include "board.h"

void setup_led(void) {
    LED_TRIS = 0; // Set RA5 as output
    LED_ANSEL = 0; // Disable analog function on RA5
}
void led_on(void) {
    LED_LAT = 1; // Turn on LED
}
void led_off(void) {
    LED_LAT = 0; // Turn off LED
}
void led_toggle(void) {
    LED_LAT = !LED_LAT; // Toggle LED
}