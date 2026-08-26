#include <xc.h>
#include "uart.h"
#include "board.h"

#define _XTAL_FREQ 64000000UL
#define BAUD_RATE   9600UL
// default speed range at reset: 16 clocks per bit -> n = Fosc/(16*baud) - 1
#define UART_BRG_VALUE ((_XTAL_FREQ + 8UL*BAUD_RATE) / (16UL*BAUD_RATE) - 1UL)

void UART_Init(void) {
    // ---- pins: route UART1 onto RA1 (TX) / RA3 (RX) ----
    UART_TX_TRIS  = 0x0;
    UART_RX_TRIS  = 0x1;
    UART_TX_ANSEL = 0x0;
    UART_RX_ANSEL = 0x0;

    RA1PPS  = 0x20;   // UART1 TX output onto RA1
    U1RXPPS = 0x03;   // UART1 RX input from RA3

    // ---- peripheral: baud rate, then enable ----
    UART_BAUD_H = (uint8_t)(UART_BRG_VALUE >> 8);
    UART_BAUD_L = (uint8_t)(UART_BRG_VALUE & 0xFF);

    UART_TX_ENABLE = 1;
    UART_RX_ENABLE = 1;
    UART_ENABLE    = 1;   // turn the whole module on last
}

void uart_send_byte(uint8_t data) {
    while (!UART_TX_READY);
    UART_TX_REG = data;
}

void uart_send_string(const char *str) {
    while (*str) {
        uart_send_byte((uint8_t)(*str++));
    }
}

uint8_t uart_data_ready(void) {
    return UART_RX_READY;
}

uint8_t uart_read_byte(void) {
    return UART_RX_REG;
}