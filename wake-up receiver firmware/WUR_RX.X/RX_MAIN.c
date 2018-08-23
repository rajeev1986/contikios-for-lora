/*---------------------------------------------------------------------------*/
/* 
 * File:   RX_MAIN.c
 * Author: Rajeev Piyare (rajeev.piyare@hotmail.com)
 * Wake-up receiver firmware for PIC12LF1552 with address decoding feature
 * Created on April 13 2018
 */
/*---------------------------------------------------------------------------*/
#include <htc.h>
#include "hardware_config.h"
#include "receiver.h"
#include <pic12lf1552.h>

#define _XTAL_FREQ 2000000  // 2MHz MCU clock frequency
#define PACKET_SIZE 2       //wake-up beacon 2BYTE packet

//FIRST 2 BITs MUST BE 10 to decode address properly
//wake-up receiver address
#define NODE_ADDRESS     0xA500  
#define WUPSIG      RA2 = 1;__delay_ms(1000);RA2 = 0; //Interrupt PIN to MSP430

unsigned char bitnr;
unsigned char bytenr;
unsigned char dataPacket[10];
unsigned char STEPCNT; 
enum state_type State;
/*---------------------------------------------------------------------------*/
static void interrupt int0()
{    
    if (( RFINTF ) && (State==IDLE)){
      RFINTF = 0;
      State = RECEIVE;
    }
    else if (( RFINTF ) && (State!=IDLE)){
        RFINTF = 0;
        State = IDLE;
    }
    if ( TMR0IF ){
        TMR0IF = 0;
        STEPCNT=1;
    }
    if ( IOCAF1 ){
        IOCAF1 = 0;
    }
}
/*---------------------------------------------------------------------------*/
void main(){
    board_init();   //initialize the WUR module
    unsigned short address_received;
    TRISA2 = 0;     //set PORT RA2 as output  
    GIE = 1;        //enable global interrupts
    PEIE=1;         //enable peripheral interrupts
    State=IDLE;     //put MCU to sleep state (IDLE)
    IOCIE=1;

while(1) {    
    switch(State)
    {
        case IDLE: 
            SLEEP(); //put PIC MCU to sleep unless RF signal detected
            break;

        case RECEIVE:
            address_received=Receive_Packet();
            //address matching logic
                if (address_received == NODE_ADDRESS){
                    //if its a match, trigger GPIO HIGH
                    WUPSIG 
                }
                else State=IDLE;
            break;
        default: State=IDLE; break;
    }
}
    
}
/*---------------------------------------------------------------------------*/