#include <htc.h>
#include "receiver.h"
#include "hardware_config.h"
#include <pic.h>

extern enum state_type State;
unsigned char rec_vect[16];
int rec_i;
unsigned int out;
unsigned char temp;             //global declared to reduce latency
extern unsigned char STEPCNT;   //Needed to count 1ms, used in ISR

unsigned short Receive_Packet()
{
    
    while(DATA_IN==1);  //wait first descent slope
    __delay_us(200);    //could be improved using timers 
    
    for(rec_i=1;rec_i<16;rec_i++)
    {
        TMR0=32;
        STEPCNT=0;
        rec_vect[rec_i]=DATA_IN;
        while(!STEPCNT);
        STEPCNT=0;
        
    }
    rec_vect[0]=1;
    out=0;
    for(rec_i=15;rec_i>=0;rec_i--)
        out|=(rec_vect[15-rec_i])<<(rec_i);
    return out;
}