/*
 * sx1276-config.h
 *
 *  Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Created : 2018-02-28
 */
/*---------------------------------------------------------------------------*/
#ifndef __SX1276_CONFIG_H__
#define __SX1276_CONFIG_H__
/*---------------------------------------------------------------------------*/
#define LORA_MAX_PAYLOAD_SIZE			255		// Max payload size in bytes
#define TX_TIMEOUT_VALUE				3000000		// us

#define CCA_THRESHOLD					-120.0		// Clear-Channel Assessment Threshold (dBm)
#define RX_TIMEOUT_VALUE				0		// us (0 = Continuously listening)

#define RX_CONTINUOUS_MODE			   	(RX_TIMEOUT_VALUE ? false : true)
/*---------------------------------------------------------------------------*/
#define RF_FREQUENCY				   	868000000	// Hz

// #define FSK_FDEV                       	25e3      // Hz
// #define FSK_DATARATE                   	50e3      // bps
// #define FSK_BANDWIDTH                  	50e3      // Hz
// #define FSK_AFC_BANDWIDTH              	83.333e3  // Hz
// #define FSK_PREAMBLE_LENGTH            	5         // Same for Tx and Rx
// #define FSK_FIX_LENGTH_PAYLOAD_ON      	false

#define TX_OUTPUT_POWER					14		// dBm
#define LORA_FDEV 						0
#define LORA_BANDWIDTH                 	2      	// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR          	12     	// [SF7..SF12]
#define LORA_CODINGRATE                	2     	// [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH           	8     	// Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON     	false
#define LORA_CRC_ON                     true 
#define LORA_FREQ_HOP    				false 				
#define LORA_FREQ_HOP_PERIOD			0
#define LORA_IQ_INVERSION_ON           	false
#define LORA_SYMBOL_TIMEOUT            	3000      	// Symbols    (3000)
#define LORA_AFC_BANDWIDTH              0    	//Hz
#define LORA_PAYLOAD_LENGTH             0
// #define LORA_RX_CONTINUOUS				true

#define OOK_TX_POWER                   10       //dBm
#define OOK_FDEV                       0        //Hz
#define OOK_BANDWIDTH_TX               0        //kHz
#define OOK_BANDWIDTH_RX               50000    //kHz
#define OOK_DATARATE                   1000     //1kbps
#define OOK_CODINGRATE                 0        //
#define OOK_AFC_BANDWIDTH              83333    //Hz
#define OOK_PREAMBLE_LENGTH            0        //Same for Tx and Rx
#define OOK_RX_TIMEOUT                 21u      //milliseconds
#define OOK_FIX_LENGTH                 1        //Fixed length packets [0: variable, 1: fixed]
#define OOK_CRC_ON                     0        //Enables/Disables the CRC [0: OFF, 1: ON]
#define OOK_PAYLOAD_LENGTH             1        //Sets payload length when fixed length is used in bytes

/*---------------------------------------------------------------------------*/
#endif /* __SX1276_CONFIG_H__ */
/*---------------------------------------------------------------------------*/
