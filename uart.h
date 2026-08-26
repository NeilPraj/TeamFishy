#include<stdint.h>

void UART_Init(void);

void uart_send_string(const char *str);

void uart_send_byte(uint8_t data);

uint8_t uart_data_ready(void);

uint8_t uart_read_byte(void);