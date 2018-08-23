#include "hardware_config.h"
#include <xc.h>
#include <pic.h>

void board_init()
{
    SPLLEN = 0;
    OSCCONbits.IRCF = 0b1100;   //2MHz
    OSCCONbits.SCS = 0b00;
    __delay_ms(1);
    
    ANSELA = 0;                 //All bit digital
    WPUA = 0;                   //All pull-up disabled
    RFINTPinDir = 1;            //Input
    DATAPinDir = 1;             //Input
    
    TRISA0 = 0;     //output
    TRISA1 = 1;     //input
    
    RFINT_POSITIVE_EDGE = 1;
    RFINT_NEGATIVE_EDGE = 0;
    tmr0_init();                                                           
}

void tmr0_init(){
    TMR0CS = 0;     //Internal instruction clock
    PSA = 0;        //Prescaler enabled
    PS0 = 0;        //1:2 -> 4us timestep, overflow 1,024 ms
    PS1 = 0;
    PS2 = 0;

    TMR0IF = 0;
    TMR0IE = 1;     //Int enabled
}
