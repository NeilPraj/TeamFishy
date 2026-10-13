/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.1.1
*/

/*
© [2026] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set RA0 aliases
#define POT1_TRIS                 TRISAbits.TRISA0
#define POT1_LAT                  LATAbits.LATA0
#define POT1_PORT                 PORTAbits.RA0
#define POT1_WPU                  WPUAbits.WPUA0
#define POT1_OD                   ODCONAbits.ODCA0
#define POT1_ANS                  ANSELAbits.ANSELA0
#define POT1_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define POT1_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define POT1_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define POT1_GetValue()           PORTAbits.RA0
#define POT1_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define POT1_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define POT1_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define POT1_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define POT1_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define POT1_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define POT1_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define POT1_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set RA1 aliases
#define POT2_TRIS                 TRISAbits.TRISA1
#define POT2_LAT                  LATAbits.LATA1
#define POT2_PORT                 PORTAbits.RA1
#define POT2_WPU                  WPUAbits.WPUA1
#define POT2_OD                   ODCONAbits.ODCA1
#define POT2_ANS                  ANSELAbits.ANSELA1
#define POT2_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define POT2_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define POT2_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define POT2_GetValue()           PORTAbits.RA1
#define POT2_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define POT2_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define POT2_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define POT2_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define POT2_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define POT2_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define POT2_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define POT2_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set RA2 aliases
#define POT3_TRIS                 TRISAbits.TRISA2
#define POT3_LAT                  LATAbits.LATA2
#define POT3_PORT                 PORTAbits.RA2
#define POT3_WPU                  WPUAbits.WPUA2
#define POT3_OD                   ODCONAbits.ODCA2
#define POT3_ANS                  ANSELAbits.ANSELA2
#define POT3_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define POT3_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define POT3_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define POT3_GetValue()           PORTAbits.RA2
#define POT3_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define POT3_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define POT3_SetPullup()          do { WPUAbits.WPUA2 = 1; } while(0)
#define POT3_ResetPullup()        do { WPUAbits.WPUA2 = 0; } while(0)
#define POT3_SetPushPull()        do { ODCONAbits.ODCA2 = 0; } while(0)
#define POT3_SetOpenDrain()       do { ODCONAbits.ODCA2 = 1; } while(0)
#define POT3_SetAnalogMode()      do { ANSELAbits.ANSELA2 = 1; } while(0)
#define POT3_SetDigitalMode()     do { ANSELAbits.ANSELA2 = 0; } while(0)

// get/set RA3 aliases
#define SWN_TRIS                 TRISAbits.TRISA3
#define SWN_LAT                  LATAbits.LATA3
#define SWN_PORT                 PORTAbits.RA3
#define SWN_WPU                  WPUAbits.WPUA3
#define SWN_OD                   ODCONAbits.ODCA3
#define SWN_ANS                  ANSELAbits.ANSELA3
#define SWN_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define SWN_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define SWN_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define SWN_GetValue()           PORTAbits.RA3
#define SWN_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define SWN_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define SWN_SetPullup()          do { WPUAbits.WPUA3 = 1; } while(0)
#define SWN_ResetPullup()        do { WPUAbits.WPUA3 = 0; } while(0)
#define SWN_SetPushPull()        do { ODCONAbits.ODCA3 = 0; } while(0)
#define SWN_SetOpenDrain()       do { ODCONAbits.ODCA3 = 1; } while(0)
#define SWN_SetAnalogMode()      do { ANSELAbits.ANSELA3 = 1; } while(0)
#define SWN_SetDigitalMode()     do { ANSELAbits.ANSELA3 = 0; } while(0)

// get/set RA4 aliases
#define SERIN_TRIS                 TRISAbits.TRISA4
#define SERIN_LAT                  LATAbits.LATA4
#define SERIN_PORT                 PORTAbits.RA4
#define SERIN_WPU                  WPUAbits.WPUA4
#define SERIN_OD                   ODCONAbits.ODCA4
#define SERIN_ANS                  ANSELAbits.ANSELA4
#define SERIN_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define SERIN_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define SERIN_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define SERIN_GetValue()           PORTAbits.RA4
#define SERIN_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define SERIN_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define SERIN_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define SERIN_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define SERIN_SetPushPull()        do { ODCONAbits.ODCA4 = 0; } while(0)
#define SERIN_SetOpenDrain()       do { ODCONAbits.ODCA4 = 1; } while(0)
#define SERIN_SetAnalogMode()      do { ANSELAbits.ANSELA4 = 1; } while(0)
#define SERIN_SetDigitalMode()     do { ANSELAbits.ANSELA4 = 0; } while(0)

// get/set RA5 aliases
#define SCLK_TRIS                 TRISAbits.TRISA5
#define SCLK_LAT                  LATAbits.LATA5
#define SCLK_PORT                 PORTAbits.RA5
#define SCLK_WPU                  WPUAbits.WPUA5
#define SCLK_OD                   ODCONAbits.ODCA5
#define SCLK_ANS                  ANSELAbits.ANSELA5
#define SCLK_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define SCLK_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define SCLK_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define SCLK_GetValue()           PORTAbits.RA5
#define SCLK_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define SCLK_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define SCLK_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define SCLK_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define SCLK_SetPushPull()        do { ODCONAbits.ODCA5 = 0; } while(0)
#define SCLK_SetOpenDrain()       do { ODCONAbits.ODCA5 = 1; } while(0)
#define SCLK_SetAnalogMode()      do { ANSELAbits.ANSELA5 = 1; } while(0)
#define SCLK_SetDigitalMode()     do { ANSELAbits.ANSELA5 = 0; } while(0)

// get/set RA6 aliases
#define RCLK_TRIS                 TRISAbits.TRISA6
#define RCLK_LAT                  LATAbits.LATA6
#define RCLK_PORT                 PORTAbits.RA6
#define RCLK_WPU                  WPUAbits.WPUA6
#define RCLK_OD                   ODCONAbits.ODCA6
#define RCLK_ANS                  ANSELAbits.ANSELA6
#define RCLK_SetHigh()            do { LATAbits.LATA6 = 1; } while(0)
#define RCLK_SetLow()             do { LATAbits.LATA6 = 0; } while(0)
#define RCLK_Toggle()             do { LATAbits.LATA6 = ~LATAbits.LATA6; } while(0)
#define RCLK_GetValue()           PORTAbits.RA6
#define RCLK_SetDigitalInput()    do { TRISAbits.TRISA6 = 1; } while(0)
#define RCLK_SetDigitalOutput()   do { TRISAbits.TRISA6 = 0; } while(0)
#define RCLK_SetPullup()          do { WPUAbits.WPUA6 = 1; } while(0)
#define RCLK_ResetPullup()        do { WPUAbits.WPUA6 = 0; } while(0)
#define RCLK_SetPushPull()        do { ODCONAbits.ODCA6 = 0; } while(0)
#define RCLK_SetOpenDrain()       do { ODCONAbits.ODCA6 = 1; } while(0)
#define RCLK_SetAnalogMode()      do { ANSELAbits.ANSELA6 = 1; } while(0)
#define RCLK_SetDigitalMode()     do { ANSELAbits.ANSELA6 = 0; } while(0)

// get/set RA7 aliases
#define SEROUT_TRIS                 TRISAbits.TRISA7
#define SEROUT_LAT                  LATAbits.LATA7
#define SEROUT_PORT                 PORTAbits.RA7
#define SEROUT_WPU                  WPUAbits.WPUA7
#define SEROUT_OD                   ODCONAbits.ODCA7
#define SEROUT_ANS                  ANSELAbits.ANSELA7
#define SEROUT_SetHigh()            do { LATAbits.LATA7 = 1; } while(0)
#define SEROUT_SetLow()             do { LATAbits.LATA7 = 0; } while(0)
#define SEROUT_Toggle()             do { LATAbits.LATA7 = ~LATAbits.LATA7; } while(0)
#define SEROUT_GetValue()           PORTAbits.RA7
#define SEROUT_SetDigitalInput()    do { TRISAbits.TRISA7 = 1; } while(0)
#define SEROUT_SetDigitalOutput()   do { TRISAbits.TRISA7 = 0; } while(0)
#define SEROUT_SetPullup()          do { WPUAbits.WPUA7 = 1; } while(0)
#define SEROUT_ResetPullup()        do { WPUAbits.WPUA7 = 0; } while(0)
#define SEROUT_SetPushPull()        do { ODCONAbits.ODCA7 = 0; } while(0)
#define SEROUT_SetOpenDrain()       do { ODCONAbits.ODCA7 = 1; } while(0)
#define SEROUT_SetAnalogMode()      do { ANSELAbits.ANSELA7 = 1; } while(0)
#define SEROUT_SetDigitalMode()     do { ANSELAbits.ANSELA7 = 0; } while(0)

// get/set RB0 aliases
#define RTMOT1_TRIS                 TRISBbits.TRISB0
#define RTMOT1_LAT                  LATBbits.LATB0
#define RTMOT1_PORT                 PORTBbits.RB0
#define RTMOT1_WPU                  WPUBbits.WPUB0
#define RTMOT1_OD                   ODCONBbits.ODCB0
#define RTMOT1_ANS                  ANSELBbits.ANSELB0
#define RTMOT1_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define RTMOT1_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define RTMOT1_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define RTMOT1_GetValue()           PORTBbits.RB0
#define RTMOT1_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define RTMOT1_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define RTMOT1_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define RTMOT1_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)
#define RTMOT1_SetPushPull()        do { ODCONBbits.ODCB0 = 0; } while(0)
#define RTMOT1_SetOpenDrain()       do { ODCONBbits.ODCB0 = 1; } while(0)
#define RTMOT1_SetAnalogMode()      do { ANSELBbits.ANSELB0 = 1; } while(0)
#define RTMOT1_SetDigitalMode()     do { ANSELBbits.ANSELB0 = 0; } while(0)

// get/set RB1 aliases
#define RTMOT2_TRIS                 TRISBbits.TRISB1
#define RTMOT2_LAT                  LATBbits.LATB1
#define RTMOT2_PORT                 PORTBbits.RB1
#define RTMOT2_WPU                  WPUBbits.WPUB1
#define RTMOT2_OD                   ODCONBbits.ODCB1
#define RTMOT2_ANS                  ANSELBbits.ANSELB1
#define RTMOT2_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define RTMOT2_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define RTMOT2_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define RTMOT2_GetValue()           PORTBbits.RB1
#define RTMOT2_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define RTMOT2_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define RTMOT2_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define RTMOT2_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define RTMOT2_SetPushPull()        do { ODCONBbits.ODCB1 = 0; } while(0)
#define RTMOT2_SetOpenDrain()       do { ODCONBbits.ODCB1 = 1; } while(0)
#define RTMOT2_SetAnalogMode()      do { ANSELBbits.ANSELB1 = 1; } while(0)
#define RTMOT2_SetDigitalMode()     do { ANSELBbits.ANSELB1 = 0; } while(0)

// get/set RB2 aliases
#define LFTMOT1_TRIS                 TRISBbits.TRISB2
#define LFTMOT1_LAT                  LATBbits.LATB2
#define LFTMOT1_PORT                 PORTBbits.RB2
#define LFTMOT1_WPU                  WPUBbits.WPUB2
#define LFTMOT1_OD                   ODCONBbits.ODCB2
#define LFTMOT1_ANS                  ANSELBbits.ANSELB2
#define LFTMOT1_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define LFTMOT1_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define LFTMOT1_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define LFTMOT1_GetValue()           PORTBbits.RB2
#define LFTMOT1_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define LFTMOT1_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define LFTMOT1_SetPullup()          do { WPUBbits.WPUB2 = 1; } while(0)
#define LFTMOT1_ResetPullup()        do { WPUBbits.WPUB2 = 0; } while(0)
#define LFTMOT1_SetPushPull()        do { ODCONBbits.ODCB2 = 0; } while(0)
#define LFTMOT1_SetOpenDrain()       do { ODCONBbits.ODCB2 = 1; } while(0)
#define LFTMOT1_SetAnalogMode()      do { ANSELBbits.ANSELB2 = 1; } while(0)
#define LFTMOT1_SetDigitalMode()     do { ANSELBbits.ANSELB2 = 0; } while(0)

// get/set RB3 aliases
#define LFTMOT2_TRIS                 TRISBbits.TRISB3
#define LFTMOT2_LAT                  LATBbits.LATB3
#define LFTMOT2_PORT                 PORTBbits.RB3
#define LFTMOT2_WPU                  WPUBbits.WPUB3
#define LFTMOT2_OD                   ODCONBbits.ODCB3
#define LFTMOT2_ANS                  ANSELBbits.ANSELB3
#define LFTMOT2_SetHigh()            do { LATBbits.LATB3 = 1; } while(0)
#define LFTMOT2_SetLow()             do { LATBbits.LATB3 = 0; } while(0)
#define LFTMOT2_Toggle()             do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define LFTMOT2_GetValue()           PORTBbits.RB3
#define LFTMOT2_SetDigitalInput()    do { TRISBbits.TRISB3 = 1; } while(0)
#define LFTMOT2_SetDigitalOutput()   do { TRISBbits.TRISB3 = 0; } while(0)
#define LFTMOT2_SetPullup()          do { WPUBbits.WPUB3 = 1; } while(0)
#define LFTMOT2_ResetPullup()        do { WPUBbits.WPUB3 = 0; } while(0)
#define LFTMOT2_SetPushPull()        do { ODCONBbits.ODCB3 = 0; } while(0)
#define LFTMOT2_SetOpenDrain()       do { ODCONBbits.ODCB3 = 1; } while(0)
#define LFTMOT2_SetAnalogMode()      do { ANSELBbits.ANSELB3 = 1; } while(0)
#define LFTMOT2_SetDigitalMode()     do { ANSELBbits.ANSELB3 = 0; } while(0)

// get/set RB4 aliases
#define PICTX_TRIS                 TRISBbits.TRISB4
#define PICTX_LAT                  LATBbits.LATB4
#define PICTX_PORT                 PORTBbits.RB4
#define PICTX_WPU                  WPUBbits.WPUB4
#define PICTX_OD                   ODCONBbits.ODCB4
#define PICTX_ANS                  ANSELBbits.ANSELB4
#define PICTX_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define PICTX_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define PICTX_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define PICTX_GetValue()           PORTBbits.RB4
#define PICTX_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define PICTX_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define PICTX_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define PICTX_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
#define PICTX_SetPushPull()        do { ODCONBbits.ODCB4 = 0; } while(0)
#define PICTX_SetOpenDrain()       do { ODCONBbits.ODCB4 = 1; } while(0)
#define PICTX_SetAnalogMode()      do { ANSELBbits.ANSELB4 = 1; } while(0)
#define PICTX_SetDigitalMode()     do { ANSELBbits.ANSELB4 = 0; } while(0)

// get/set RB5 aliases
#define PICRX_TRIS                 TRISBbits.TRISB5
#define PICRX_LAT                  LATBbits.LATB5
#define PICRX_PORT                 PORTBbits.RB5
#define PICRX_WPU                  WPUBbits.WPUB5
#define PICRX_OD                   ODCONBbits.ODCB5
#define PICRX_ANS                  ANSELBbits.ANSELB5
#define PICRX_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define PICRX_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define PICRX_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define PICRX_GetValue()           PORTBbits.RB5
#define PICRX_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define PICRX_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define PICRX_SetPullup()          do { WPUBbits.WPUB5 = 1; } while(0)
#define PICRX_ResetPullup()        do { WPUBbits.WPUB5 = 0; } while(0)
#define PICRX_SetPushPull()        do { ODCONBbits.ODCB5 = 0; } while(0)
#define PICRX_SetOpenDrain()       do { ODCONBbits.ODCB5 = 1; } while(0)
#define PICRX_SetAnalogMode()      do { ANSELBbits.ANSELB5 = 1; } while(0)
#define PICRX_SetDigitalMode()     do { ANSELBbits.ANSELB5 = 0; } while(0)

// get/set RB6 aliases
#define LEDN_TRIS                 TRISBbits.TRISB6
#define LEDN_LAT                  LATBbits.LATB6
#define LEDN_PORT                 PORTBbits.RB6
#define LEDN_WPU                  WPUBbits.WPUB6
#define LEDN_OD                   ODCONBbits.ODCB6
#define LEDN_ANS                  ANSELBbits.ANSELB6
#define LEDN_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define LEDN_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define LEDN_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define LEDN_GetValue()           PORTBbits.RB6
#define LEDN_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define LEDN_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define LEDN_SetPullup()          do { WPUBbits.WPUB6 = 1; } while(0)
#define LEDN_ResetPullup()        do { WPUBbits.WPUB6 = 0; } while(0)
#define LEDN_SetPushPull()        do { ODCONBbits.ODCB6 = 0; } while(0)
#define LEDN_SetOpenDrain()       do { ODCONBbits.ODCB6 = 1; } while(0)
#define LEDN_SetAnalogMode()      do { ANSELBbits.ANSELB6 = 1; } while(0)
#define LEDN_SetDigitalMode()     do { ANSELBbits.ANSELB6 = 0; } while(0)

// get/set RC0 aliases
#define CLRN_TRIS                 TRISCbits.TRISC0
#define CLRN_LAT                  LATCbits.LATC0
#define CLRN_PORT                 PORTCbits.RC0
#define CLRN_WPU                  WPUCbits.WPUC0
#define CLRN_OD                   ODCONCbits.ODCC0
#define CLRN_ANS                  ANSELCbits.ANSELC0
#define CLRN_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define CLRN_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define CLRN_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define CLRN_GetValue()           PORTCbits.RC0
#define CLRN_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define CLRN_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define CLRN_SetPullup()          do { WPUCbits.WPUC0 = 1; } while(0)
#define CLRN_ResetPullup()        do { WPUCbits.WPUC0 = 0; } while(0)
#define CLRN_SetPushPull()        do { ODCONCbits.ODCC0 = 0; } while(0)
#define CLRN_SetOpenDrain()       do { ODCONCbits.ODCC0 = 1; } while(0)
#define CLRN_SetAnalogMode()      do { ANSELCbits.ANSELC0 = 1; } while(0)
#define CLRN_SetDigitalMode()     do { ANSELCbits.ANSELC0 = 0; } while(0)

// get/set RC1 aliases
#define SENSOR1_TRIS                 TRISCbits.TRISC1
#define SENSOR1_LAT                  LATCbits.LATC1
#define SENSOR1_PORT                 PORTCbits.RC1
#define SENSOR1_WPU                  WPUCbits.WPUC1
#define SENSOR1_OD                   ODCONCbits.ODCC1
#define SENSOR1_ANS                  ANSELCbits.ANSELC1
#define SENSOR1_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define SENSOR1_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define SENSOR1_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define SENSOR1_GetValue()           PORTCbits.RC1
#define SENSOR1_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define SENSOR1_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define SENSOR1_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define SENSOR1_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define SENSOR1_SetPushPull()        do { ODCONCbits.ODCC1 = 0; } while(0)
#define SENSOR1_SetOpenDrain()       do { ODCONCbits.ODCC1 = 1; } while(0)
#define SENSOR1_SetAnalogMode()      do { ANSELCbits.ANSELC1 = 1; } while(0)
#define SENSOR1_SetDigitalMode()     do { ANSELCbits.ANSELC1 = 0; } while(0)

// get/set RC2 aliases
#define SENSOR2_TRIS                 TRISCbits.TRISC2
#define SENSOR2_LAT                  LATCbits.LATC2
#define SENSOR2_PORT                 PORTCbits.RC2
#define SENSOR2_WPU                  WPUCbits.WPUC2
#define SENSOR2_OD                   ODCONCbits.ODCC2
#define SENSOR2_ANS                  ANSELCbits.ANSELC2
#define SENSOR2_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define SENSOR2_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define SENSOR2_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define SENSOR2_GetValue()           PORTCbits.RC2
#define SENSOR2_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define SENSOR2_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define SENSOR2_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define SENSOR2_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define SENSOR2_SetPushPull()        do { ODCONCbits.ODCC2 = 0; } while(0)
#define SENSOR2_SetOpenDrain()       do { ODCONCbits.ODCC2 = 1; } while(0)
#define SENSOR2_SetAnalogMode()      do { ANSELCbits.ANSELC2 = 1; } while(0)
#define SENSOR2_SetDigitalMode()     do { ANSELCbits.ANSELC2 = 0; } while(0)

// get/set RC3 aliases
#define SENSOR3_TRIS                 TRISCbits.TRISC3
#define SENSOR3_LAT                  LATCbits.LATC3
#define SENSOR3_PORT                 PORTCbits.RC3
#define SENSOR3_WPU                  WPUCbits.WPUC3
#define SENSOR3_OD                   ODCONCbits.ODCC3
#define SENSOR3_ANS                  ANSELCbits.ANSELC3
#define SENSOR3_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define SENSOR3_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define SENSOR3_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define SENSOR3_GetValue()           PORTCbits.RC3
#define SENSOR3_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define SENSOR3_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define SENSOR3_SetPullup()          do { WPUCbits.WPUC3 = 1; } while(0)
#define SENSOR3_ResetPullup()        do { WPUCbits.WPUC3 = 0; } while(0)
#define SENSOR3_SetPushPull()        do { ODCONCbits.ODCC3 = 0; } while(0)
#define SENSOR3_SetOpenDrain()       do { ODCONCbits.ODCC3 = 1; } while(0)
#define SENSOR3_SetAnalogMode()      do { ANSELCbits.ANSELC3 = 1; } while(0)
#define SENSOR3_SetDigitalMode()     do { ANSELCbits.ANSELC3 = 0; } while(0)

// get/set RC4 aliases
#define SENSOR4_TRIS                 TRISCbits.TRISC4
#define SENSOR4_LAT                  LATCbits.LATC4
#define SENSOR4_PORT                 PORTCbits.RC4
#define SENSOR4_WPU                  WPUCbits.WPUC4
#define SENSOR4_OD                   ODCONCbits.ODCC4
#define SENSOR4_ANS                  ANSELCbits.ANSELC4
#define SENSOR4_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define SENSOR4_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define SENSOR4_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define SENSOR4_GetValue()           PORTCbits.RC4
#define SENSOR4_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define SENSOR4_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define SENSOR4_SetPullup()          do { WPUCbits.WPUC4 = 1; } while(0)
#define SENSOR4_ResetPullup()        do { WPUCbits.WPUC4 = 0; } while(0)
#define SENSOR4_SetPushPull()        do { ODCONCbits.ODCC4 = 0; } while(0)
#define SENSOR4_SetOpenDrain()       do { ODCONCbits.ODCC4 = 1; } while(0)
#define SENSOR4_SetAnalogMode()      do { ANSELCbits.ANSELC4 = 1; } while(0)
#define SENSOR4_SetDigitalMode()     do { ANSELCbits.ANSELC4 = 0; } while(0)

// get/set RC5 aliases
#define SENSOR5_TRIS                 TRISCbits.TRISC5
#define SENSOR5_LAT                  LATCbits.LATC5
#define SENSOR5_PORT                 PORTCbits.RC5
#define SENSOR5_WPU                  WPUCbits.WPUC5
#define SENSOR5_OD                   ODCONCbits.ODCC5
#define SENSOR5_ANS                  ANSELCbits.ANSELC5
#define SENSOR5_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define SENSOR5_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define SENSOR5_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define SENSOR5_GetValue()           PORTCbits.RC5
#define SENSOR5_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define SENSOR5_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define SENSOR5_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define SENSOR5_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define SENSOR5_SetPushPull()        do { ODCONCbits.ODCC5 = 0; } while(0)
#define SENSOR5_SetOpenDrain()       do { ODCONCbits.ODCC5 = 1; } while(0)
#define SENSOR5_SetAnalogMode()      do { ANSELCbits.ANSELC5 = 1; } while(0)
#define SENSOR5_SetDigitalMode()     do { ANSELCbits.ANSELC5 = 0; } while(0)

// get/set RC6 aliases
#define SENSOR6_TRIS                 TRISCbits.TRISC6
#define SENSOR6_LAT                  LATCbits.LATC6
#define SENSOR6_PORT                 PORTCbits.RC6
#define SENSOR6_WPU                  WPUCbits.WPUC6
#define SENSOR6_OD                   ODCONCbits.ODCC6
#define SENSOR6_ANS                  ANSELCbits.ANSELC6
#define SENSOR6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define SENSOR6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define SENSOR6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define SENSOR6_GetValue()           PORTCbits.RC6
#define SENSOR6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define SENSOR6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define SENSOR6_SetPullup()          do { WPUCbits.WPUC6 = 1; } while(0)
#define SENSOR6_ResetPullup()        do { WPUCbits.WPUC6 = 0; } while(0)
#define SENSOR6_SetPushPull()        do { ODCONCbits.ODCC6 = 0; } while(0)
#define SENSOR6_SetOpenDrain()       do { ODCONCbits.ODCC6 = 1; } while(0)
#define SENSOR6_SetAnalogMode()      do { ANSELCbits.ANSELC6 = 1; } while(0)
#define SENSOR6_SetDigitalMode()     do { ANSELCbits.ANSELC6 = 0; } while(0)

// get/set RC7 aliases
#define SENSOR7_TRIS                 TRISCbits.TRISC7
#define SENSOR7_LAT                  LATCbits.LATC7
#define SENSOR7_PORT                 PORTCbits.RC7
#define SENSOR7_WPU                  WPUCbits.WPUC7
#define SENSOR7_OD                   ODCONCbits.ODCC7
#define SENSOR7_ANS                  ANSELCbits.ANSELC7
#define SENSOR7_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define SENSOR7_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define SENSOR7_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define SENSOR7_GetValue()           PORTCbits.RC7
#define SENSOR7_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define SENSOR7_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define SENSOR7_SetPullup()          do { WPUCbits.WPUC7 = 1; } while(0)
#define SENSOR7_ResetPullup()        do { WPUCbits.WPUC7 = 0; } while(0)
#define SENSOR7_SetPushPull()        do { ODCONCbits.ODCC7 = 0; } while(0)
#define SENSOR7_SetOpenDrain()       do { ODCONCbits.ODCC7 = 1; } while(0)
#define SENSOR7_SetAnalogMode()      do { ANSELCbits.ANSELC7 = 1; } while(0)
#define SENSOR7_SetDigitalMode()     do { ANSELCbits.ANSELC7 = 0; } while(0)

/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);


#endif // PINS_H
/**
 End of File
*/