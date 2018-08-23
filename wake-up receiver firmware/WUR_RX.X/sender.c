
/* 
 
#include <htc.h>
#include <pic16lf1824.h>
#include "sender.h"
#include "hardware_config.h"
#include "RF_settings.h"

unsigned char *send_data;
extern unsigned char bitnr;
extern unsigned char bytenr;


void sender_interrupt()
{
    TMR4IF = 0;
    DATA = ((send_data[bytenr]>>(7-bitnr))&0x01);
    bitnr ++;
    if(bitnr==8)
    {
        bitnr = 0;
        bytenr ++;
    }
}

void send_packet(unsigned char *data, unsigned char num_bytes)
{
    send_data = data;
    // configure the transmitter
    Write_Application_Reg
    (
            T39_FORCED_TX |
            T39_MOD_OOK |
            T39_TX_10dB |
            T39_TX_2mS	|
            T39_86x_BAND
    ); 
    Write_Frequency_Reg(T39_FREQ_84385);
    DATA = 0;

    __delay_ms(1);
    bitnr = 0;
    bytenr = 0;

    // start the interrupt
    TMR4IF = 0;
    TMR4IE = 1;

    //start timer
    TMR4 = 0;
    TMR4ON = 1;

    //wait for transmission to complete
    //    DEBUG_PIN=1;                                                          //MEASUREMENTS
    while(bytenr < num_bytes);
    //    DEBUG_PIN=0;                                                    
    TMR4IE = 0;
    TMR4ON = 0;

    //shutdown transmitter
    __delay_us(1000);                                                           //inutile!
    DATA = 0;
    Write_Application_Reg(T39_POWER_AND_GO);
}


void sender_init()
{
    // All ports are digital
    ANSELC = 0;
    WPUA = 0;  //Pull-Up Disabled
    
    // Data out
    DATA_DIRECTION = 0;                                                 
    DATA = 0;

    CTRL_DIRECTION = 0;                                                 
    CTRL = 0;
    
    __delay_ms(20);   //Wait RF Sleep

    //assume 2MHz clock -> clock for timer is 500KHz
    // prescaler is 1:4 to period clk is 125KHz -> period = 8us
    T4CKPS1 = 0;
    T4CKPS0 = 1;

    //set overflow to 125 so it occurs every 1ms -> 1kbps
    PR4 = 125;

    TMR4ON = 0;

    PEIE = 1;
    GIE = 1;    
}

void Send_DATA(unsigned char cmd)
{
	char i;
	for (i=0; i<8; i++)
	{
		if (cmd & 0x80)
			DATA = 1;
		else
			DATA = 0;
	
		CTRL = 1;
		NOP();
		NOP();
		CTRL = 0;
		cmd = cmd << 1;
	}
}

void Write_Application_Reg (int cmd)
{
    Send_DATA(0);
    Send_DATA((cmd & 0xFF00) >> 8);
    Send_DATA((cmd & 0x00FF));
}

void Write_Frequency_Reg (long cmd)
{
    cmd += 0x180000;
    Send_DATA((cmd & 0xFF0000) >> 16);
    Send_DATA((cmd & 0x00FF00) >> 8);
    Send_DATA((cmd & 0x0000FF));
}


void RecoverySequenceTiming (){
    unsigned char i;
    DATA = 0;
    CTRL = 0;
    for (i=0;i<24;i++){
        DATA = 1;
        //delay t1/2
        CTRL = 1;
        //delay t1/2
        DATA = 0;
        //delay t0/2
        CTRL = 0;
        //delay t0/2
        NOP();
        
    }
}

*/