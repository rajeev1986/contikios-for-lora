/* 
 * File: hardware_config.h
 * Author: Rajeev Piyare
 * Comments: HW and PORT configuration For wake-up receiver module.
 * Revision history: 7 March 2018
 */

#ifndef HW_CONFIG
#define	HW_CONFIG
#include <xc.h>  

#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#define _XTAL_FREQ 2000000

//receiver
#define DATA_IN     RA4
#define RFINT       RA5
#define RFINTPinDir TRISA5
#define RFINTPinAn  ANSA5
#define DATAPinDir  TRISA4
#define DATAPinAn   ANSA4
#define RFINT_POSITIVE_EDGE IOCAP5
#define RFINT_NEGATIVE_EDGE IOCAN5
#define DATA_POSITIVE_EDGE  IOCAP4
#define DATA_NEGATIVE_EDGE  IOCAN4

#define RFINTF IOCAF5
#define DATAF IOCAF4

enum state_type {IDLE,RECEIVE};
/*******************************************************************************
 * @fn      board_init
 *
 * @brief   initialises the clock to a value of 2MHz, initialize Pin direction
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void board_init();


/*******************************************************************************
 * @fn      tmr0_init
 *
 * @brief   Inizialize Timer 0 as Counter Mode
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void tmr0_init();


#endif	/* XC_HEADER_TEMPLATE_H */

