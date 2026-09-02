#include <xc.h>
// LED defines
#define LED_TRIS TRISAbits.TRISA5
#define LED_ANSEL ANSELAbits.ANSELA5
#define LED_LAT LATAbits.LATA5

// ------ ADC defines ------

// ADC defines
#define ADC_ANSEL ANSELAbits.ANSELA0
#define ADC_TRIS TRISAbits.TRISA0

// ADC channel selection
#define ADC_CHANNEL_SEL ADPCH // register you write the channel value into
#define ADC_CHANNEL_1 0x00    // value: selects ANA0 RA0

// ADC clock
#define ADC_CLOCK ADCLK   // register
#define ADC_CLOCK_DIV 0x7 // value: settle time divider settings

// ADC reference voltage selection
#define ADC_REF_REG                                                            \
  ADREFbits           // struct, so you can do ADC_REF_REG.ADPREF = ADC_PREF;
#define ADC_PREF 0x00 // value: VDD as +ref
#define ADC_NREF 0x00 // value: VSS as -ref

// ADC acquisition time
#define ADC_ACQ ADACQ     // register
#define ADC_ACQ_TIME 0x05 // value: acquisition cycles before sampling

// ADC control bits
#define ADC_FORMAT ADCON0bits.ADFM0 // location
#define ADC_FORMAT_RIGHT 1          // value: right-justified result
#define ADC_ON ADCON0bits.ADON      // location
#define ADC_GO                                                                 \
  ADCON0bits.ADGO // location, also doubles as "conversion in progress" flag

// ADC result registers
#define ADC_RESULT_H ADRESH
#define ADC_RESULT_L ADRESL

// ----- UART pin routing -----
#define UART_TX_TRIS TRISBbits.TRISB5
#define UART_RX_TRIS TRISBbits.TRISB4
#define UART_TX_ANSEL ANSELBbits.ANSELB5
#define UART_RX_ANSEL ANSELBbits.ANSELB4

// ----- UART (UART1 peripheral) defines -----
#define TX_PPS                                                                 \
  RB5PPS // PPS output register belonging to the TX pin. Change for new pin
#define TX_PPS_VAL 0x20 // function code: UART1 TX

#define RX_PPS U1RXPPS  // PPS input register belonging to UART1's RX signal
#define RX_PPS_VAL 0x0C // pin code: RB4 Change for different pin

#define UART_TX_ENABLE U1TXEN // location: transmitter enable
#define UART_RX_ENABLE U1RXEN // location: receiver enable
#define UART_ENABLE                                                            \
  U1ON // location: master on/off switch for the whole peripheral

#define UART_TX_REG U1TXB // write here to send a byte
#define UART_RX_REG U1RXB // read here to get a received byte

#define UART_BAUD_H U1BRGH
#define UART_BAUD_L U1BRGL

#define UART_TX_READY                                                          \
  U1TXIF // set when there's room in the TX buffer for another byte
#define UART_RX_READY U1RXIF // set when a received byte is waiting in U1RXB
