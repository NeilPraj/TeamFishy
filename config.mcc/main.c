 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.2
 *
 * @version Package Version: 3.1.2
*/

#include "mcc_generated_files/system/system.h"
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "imu.h"
#define PRINT_TICKS 61u                 /* ~0.25 s at 4.096 ms/tick */

volatile uint8_t control_ticks = 0u;


static void control_timer_isr(void) { control_ticks++; }

static void uart_puts(const char *s)
{
    while (*s != '\0') UART1_Write((uint8_t)*s++);
}

static void uart_i32(int32_t x)
{
    char buf[12];
    uint8_t i = 0u;
    uint32_t v;

    if (x < 0L) { UART1_Write('-'); v = (uint32_t)(-x); }
    else        { v = (uint32_t)x; }

    do { buf[i++] = (char)('0' + (v % 10u)); v /= 10u; } while (v != 0u);
    while (i != 0u) UART1_Write((uint8_t)buf[--i]);
}


int main(void)
{
    SYSTEM_Initialize();

    uint8_t n;
    uint8_t print_count = 0u;
    imu_status_t st;

    SYSTEM_Initialize();
    INLVLA = 0xFFu;                      /* same as your original main */

    TMR0_PeriodMatchCallbackRegister(control_timer_isr);
    INTCON0bits.GIEH = 1;
    INTCON0bits.GIEL = 1;

    uart_puts("\r\nIMU test - keep robot still\r\n");

    st = imu_init();
    if (st == IMU_OK) {
        uart_puts("IMU OK, bias=");
        uart_i32(imu_gyro_z_bias());
        uart_puts("\r\n");
    } else if (st == IMU_ERR_WHOAMI) {
        uart_puts("FAIL: WHO_AM_I=0x");
        uart_i32(imu_whoami());          /* decimal; expect 112 (0x70) */
        uart_puts(" - check wiring/CS/power\r\n");
        while (1) { }
    } else {
        uart_puts("FAIL: calibration - robot was moving, reset and retry\r\n");
        while (1) { }
    }

    control_ticks = 0u;                  /* discard ticks that piled up during init */

    while(1)
    {
        INTCON0bits.GIEH = 0;
        n = control_ticks;
        control_ticks = 0u;
        INTCON0bits.GIEH = 1;

        if (n != 0u) {
            imu_update(n);

            if (PORTAbits.RA3 == 0) imu_yaw_reset();   /* SW2 zeroes yaw */

            print_count += n;
            if (print_count >= PRINT_TICKS) {
                print_count = 0u;
                uart_puts("Gz=");
                uart_i32(imu_gyro_z_corrected());
                uart_puts(" yaw_mdeg=");
                uart_i32(imu_yaw_mdeg());
                uart_puts("\r\n");
            }
        }
    }    
}