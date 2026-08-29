// PIC18F47Q43 Configuration Bit Settings

// --- CONFIG1 (0x300000): Oscillator selection ---
#pragma config FEXTOSC  = 0x4  // Oscillator not enabled
#pragma config RSTOSC   = 0x0  // HFINTOSC, 64 MHz, CDIV 1:1

// --- CONFIG2 (0x300001): Clock behavior ---
#pragma config CLKOUTEN = 0x1  // CLKOUT function is disabled (RA6 stays a normal I/O pin)
#pragma config PR1WAY   = 0x0  // Interrupt and peripheral priorities are reconfigurable (disable if we don't use)
#pragma config CSWEN    = 0x1  // Clock switch is enabled -- required for clk_initialize() to actually take effect
#pragma config FCMEN    = 0x0  // Fail-Safe Clock Monitor is disabled since we don't use an external clock source

// --- CONFIG3 (0x300002): Reset & interrupt behavior ---
#pragma config MCLRE    = 0x1  // MCLR pin is enabled as hardware reset (moot while LVP=1 below, which forces this regardless)
#pragma config PWRTS    = 0x3  // Power up timer is disabled. No delay on power up
#pragma config MVECEN   = 0x1  // Multi vectored interrupts enabled
#pragma config IVT1WAY  = 0x1  // Vector interrupt table lock can be written only once per reset
#pragma config LPBOREN  = 0x1  // Low power brown out reset is disabled
#pragma config BOREN    = 0x0  // Brown out reset is disabled

// --- CONFIG4 (0x300003): Programming, stack, misc ---
#pragma config BORV     = 0x3  // Brown out reset voltage set to 1.9V (moot while BOREN is disabled above)
#pragma config ZCD      = 0x1  // Zero cross detect is disabled at POR
#pragma config PPS1WAY  = 0x1  // Peripheral pin select can be written only once per reset
#pragma config STVREN   = 0x1  // Stack overflow or underflow will cause a reset
#pragma config LVP      = 0x1  // Low voltage programming enabled
#pragma config XINST    = 0x1  // Legacy instruction set (Extended Instruction Set disabled) -- required for XC8 C code

// --- CONFIG5 (0x300004): Watchdog Timer ---
#pragma config WDTE     = 0x0  // Watchdog timer disabled

// CONFIG6 (WDTCPS/WDTCWS/WDTCCS) intentionally left unset -- irrelevant while WDTE = 0.
// CONFIG7-CONFIG10 intentionally omitted: DEBUG is tool-managed (never set by hand), and
// SAFEN/BBEN/WRT*/CP already default to their "unprotected" state, which is correct for
// development. 