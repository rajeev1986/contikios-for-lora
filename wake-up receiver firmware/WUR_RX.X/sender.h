/* 
 * File:   sender.h
 * Author: Antonino Faraone
 *
 * Created on 7 marzo 2018, 17.47
 */

#ifndef SENDER_H
#define	SENDER_H


/*******************************************************************************
 * @fn      sender_interrupt
 *
 * @brief   interrupt for sender, has to be called in interrupt routine
 *          in if(TMR4IF)
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void sender_interrupt();

/*******************************************************************************
 * @fn      send_packet
 *
 * @brief   sends a packet via RF
 *
 * @param   unsigned char *data: pointer to data to be sent
 *          unsigned char num_bytes: number of bytes at given location to be sent
 *
 * @return  none
 ******************************************************************************/
void send_packet(unsigned char *data, unsigned char num_bytes);

/*******************************************************************************
 * @fn      sender_init
 *
 * @brief   initialises the sender of the pic
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void sender_init();

/**
 * Don't use to trasmit data, only use to write on Application and Frequency 
 * register! use send_packet to send a packet
 * 
 */
void Send_DATA(unsigned char cmd); //EX sendByteTxCommand(unsigned char cmd)

void Write_Application_Reg (int cmd); //EX sendTxCommand_appl

void Write_Frequency_Reg (long cmd); //EX sendTxCommand_freq
/*******************************************************************************
 * @fn      RecoverySequenceTiming
 *
 * @brief   recovery the communication with RF bus
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void RecoverySequenceTiming ();








#endif	/* SENDER_H */

