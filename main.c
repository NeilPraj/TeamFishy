/**
 * @file main.c
 * @author cappy
 * @date 2026-08-21
 * @brief Main function
 */
#include <xc.h>
#define _XTAL_FREQ 64000000UL


int main(){

    // Add your code here and press Ctrl + Shift + B to build
     TRISBbits.TRISB0 = 0;
    while(1) {
        LATBbits.LATB0 = 1;
    }

    return 0;
}
