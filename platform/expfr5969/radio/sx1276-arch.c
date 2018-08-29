/*
 * Copyright (c) 2018, Bruno Kessler Foundation, Trento, Italy and
 * ETH, IIS, Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * \file
 *          sx1276-arch.c that holds the LoRa radio abstraction Layer for ContikiOS
 * \author
 *          Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Updated : 2018-04-04
 */
/*---------------------------------------------------------------------------
Description: Contiki radio interface implementation for SX1276 Driver
-----------------------------------------------------------------------------*/                                                             
#include "contiki.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"
#include "sys/energest.h"
#include "sx1276-arch.h"
#include "sx1276-config.h"
#include "sx1276.h"
#include "spi.h"
#include <string.h>
#include <stdio.h>
#include "sys/clock.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif
/*---------------------------------------------------------------------------*/
int8_t rx_last_snr = 0;
int16_t rx_last_rssi = 0;

// Set current params to initial value
uint8_t sx1276_sf       = LORA_SPREADING_FACTOR;
uint8_t sx1276_bw       = LORA_BANDWIDTH;
uint8_t sx1276_cr       = LORA_CODINGRATE;
uint8_t sx1276_tx_power = TX_OUTPUT_POWER;

/* Packet Buffer setting*/
static uint8_t rx_msg_buf[RX_BUFFER_SIZE];
static uint16_t rx_msg_size = 0;

static int tx_ongoing =0;
static int pending_packets = 0;
static int packet_is_prepared = 0;
static RadioEvents_t RadioEvents;
// static struct etimer et_reset_rx;
//static packetbuf_attr_t last_rssi = 0;

/*---------------------------------------------------------------------------*/
/* SX1276 Radio Driver Static Functions */
static int sx1276_radio_init(void);
static int sx1276_radio_on(void);
static int sx1276_radio_off(void);
static int sx1276_radio_prepare(const void *payload, unsigned short payload_len);
static int sx1276_radio_transmit(unsigned short payload_len);
static int sx1276_radio_send(const void *data, unsigned short len);
static int sx1276_radio_read(void *buf, unsigned short bufsize);
//static int sx1276_radio_channel_clear(void);
static int sx1276_radio_receiving_packet(void);
static int sx1276_radio_pending_packet(void);
/*---------------------------------------------------------------------------*/
PROCESS(sx1276_process, "SX1276 driver");
/*---------------------------------------------------------------------------*/
void OnTxDone( void )
{
	PRINTF("TX Done\n");
  tx_ongoing =0;
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  /* Enable RX */
  sx1276_set_rx(RX_TIMEOUT_VALUE);
}
/*---------------------------------------------------------------------------*/
void OnRxDone( uint8_t *payload, uint16_t size, int8_t rssi, int8_t snr )
{
	rx_msg_size = size;
	memcpy(rx_msg_buf, payload, rx_msg_size);
	pending_packets ++;

   // save Rssi and SNR
  rx_last_snr = snr;
  rx_last_rssi = rssi;
	 
	printf("Incoming MSG: Size: %d bytes, RSSI: %d, SNR: %d\n", rx_msg_size, rssi, snr);

  process_poll(&sx1276_process);
  sx1276_radio_on();
	
}
/*---------------------------------------------------------------------------*/
void OnTxTimeout( void )
{
	PRINTF("TX timeout\n");
	tx_ongoing =0;
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  //sx1276_set_rx(RX_TIMEOUT_VALUE);
  sx1276_set_sleep();
}
/*---------------------------------------------------------------------------*/
void OnRxTimeout( void )
{
  PRINTF("RX Timeout\n");
  sx1276_set_rx(RX_TIMEOUT_VALUE);
  // Radio.Sleep();
  sx1276_set_sleep();
}
/*---------------------------------------------------------------------------*/
void OnRxError( void )
{
	PRINTF("RX Error\n");
  // sx1276_set_rx(RX_TIMEOUT_VALUE);
  // Radio.Sleep();
  sx1276_set_sleep();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sx1276_process, ev, data)
{
  uint8_t len;
  PROCESS_BEGIN();

  while(1)
  {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

		PRINTF("LoRa radio: polled\n");
    sx1276_driver.on();

    /* Clear packetbuf to avoid leftovers from previous RX */
    packetbuf_clear();

    /* Copy the received frame to packetbuf */
    len = sx1276_radio_read(packetbuf_dataptr(), PACKETBUF_SIZE);
    packetbuf_set_datalen(len);
    /* Turn on radio to keep listening */
    sx1276_radio_on();
    NETSTACK_RDC.input();
    
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*                        CONTIKI INTERFACE                                  */
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_init(void)
{
  PRINTF("\nInitializing sx1276\n");

  spi_init();

  // Radio initialization
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
	
  sx1276_init(&RadioEvents);

  sx1276_set_channel( RF_FREQUENCY );

  sx1276_set_txconfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, LORA_IQ_INVERSION_ON, 3000);


  sx1276_set_rxconfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                  0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  sx1276_sf       = LORA_SPREADING_FACTOR;
  sx1276_bw       = LORA_BANDWIDTH;
  sx1276_cr       = LORA_CODINGRATE;
  sx1276_tx_power = TX_OUTPUT_POWER;

  printf("SX1276 initialized with Freq:%luHz, TX Pwr: %ddBm, BW:%d, SF:%d, CR:%d\n\r",
          RF_FREQUENCY, TX_OUTPUT_POWER, LORA_BANDWIDTH,
          LORA_SPREADING_FACTOR, LORA_CODINGRATE); 

	pending_packets = 0;
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  process_start(&sx1276_process, NULL);
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_on(void)
{
  /* Enable RX */
  sx1276_set_rx(RX_TIMEOUT_VALUE);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  PRINTF("Radio ON\n");
	// Start reset timer
	 // etimer_set(&et_reset_rx, RESET_RX_DURATION );
	 // // assign it to correct process
	 // et_reset_rx.p = &sx1276_process;
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_off(void)
{
	// stop reset timer
	// etimer_stop(&et_reset_rx); 
  sx1276_set_rx(RX_TIMEOUT_VALUE); //need timers
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  PRINTF("Radio off\n");
  //sx1276_set_sleep();
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_read(void *buf, unsigned short bufsize)
{
	if (pending_packets > 0) {
	  pending_packets --;

	  if (bufsize < rx_msg_size) {
			PRINTF("WARNING: Buffer size is small\n\r");
			 return 0;
		}
		else {
      RIMESTATS_ADD(llrx);
			memcpy(buf, rx_msg_buf, rx_msg_size);
      //packetbuf_set_attr(PACKETBUF_ATTR_RSSI, last_rssi);
			return rx_msg_size;
		}
	}
  return 0;
}
/*---------------------------------------------------------------------------*/
// static int
// sx1276_radio_channel_clear(void)
// {
//   bool channel_clear;
//     //TODO: FIX!
//   channel_clear = 1;

//   PRINTF("CHANNEL CLEAR %d\n", channel_clear);
//   return channel_clear;
// 	//return Radio.IsChannelFree(MODEM_LORA, RF_FREQUENCY, LORA_CLEAR_CHANNEL_RSSI_THRESHOLD);
// }
/*---------------------------------------------------------------------------*/
static uint8_t *packet_payload = NULL;
static uint16_t packet_payload_len = 0;

static int
sx1276_radio_prepare(const void *payload, unsigned short payload_len)
{
   /* Checks if the payload length is supported */
  if(payload_len > RX_BUFFER_SIZE) {
    packet_is_prepared = 0;
    return RADIO_TX_ERR;
  }

  RIMESTATS_ADD(lltx);

  packet_payload = (uint8_t *) payload;
  packet_payload_len = payload_len;
  packet_is_prepared = 1;
  PRINTF("Payload prepared in: %u bytes\n", payload_len);

  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_transmit(unsigned short payload_len)
{
  sx1276_send(packet_payload, packet_payload_len);
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
	tx_ongoing =1;
  packet_is_prepared = 0;

  PRINTF("Transmission ended\n");
  process_poll(&sx1276_process);
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_send(const void *payload, unsigned short payload_len)
{
	sx1276_radio_prepare(payload, payload_len);
	return sx1276_radio_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_receiving_packet(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
sx1276_radio_pending_packet(void)
{
  PRINTF("Pending packet\n");
  return pending_packets;
}
/*---------------------------------------------------------------------------*/
const struct radio_driver sx1276_driver =
{
  sx1276_radio_init,
  sx1276_radio_prepare,
  sx1276_radio_transmit,
  sx1276_radio_send,
  sx1276_radio_read,
  //sx1276_radio_channel_clear,
  sx1276_radio_receiving_packet,
  sx1276_radio_pending_packet,
  sx1276_radio_on,
  sx1276_radio_off,
};
/*---------------------------------------------------------------------------*/
