#include <xc.h>
#include "clock.h"

void clk_initialize(void) {
    OSCFRQ = 0x08;                  // HFINTOSC tap frequency 64 MHz
    OSCCON1bits.NOSC = 0b110;       // New oscillator source = HFINTOSC
    OSCCON1bits.NDIV = 0b0000;      // clock = 1:1 )
    while (OSCCON3bits.ORDY == 0);  // wait until the switch has completed
}