/**
 * RI-MAC for broadcast Mode 11 April 2018
 * Sends a single broadcast wake-up signal to collect data 
 */

#include "net/mac/mac-sequence.h"
#include "net/mac/wurmac/wrimac-broadcast.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "wur.h"
#include "net/rime/rimestats.h"
#include <string.h>
#include <stdio.h>
#include "dev/leds.h"
#include "sys/rtimer.h" 
#include "node-id.h" 

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef NULLRDC_CONF_ADDRESS_FILTER
#define NULLRDC_ADDRESS_FILTER NULLRDC_CONF_ADDRESS_FILTER
#else
#define NULLRDC_ADDRESS_FILTER 1
#endif /* NULLRDC_CONF_ADDRESS_FILTER */

#ifndef NULLRDC_802154_AUTOACK
#ifdef NULLRDC_CONF_802154_AUTOACK
#define NULLRDC_802154_AUTOACK NULLRDC_CONF_802154_AUTOACK
#else
#define NULLRDC_802154_AUTOACK 0
#endif /* NULLRDC_CONF_802154_AUTOACK */
#endif /* NULLRDC_802154_AUTOACK */

#ifndef NULLRDC_802154_AUTOACK_HW
#ifdef NULLRDC_CONF_802154_AUTOACK_HW
#define NULLRDC_802154_AUTOACK_HW NULLRDC_CONF_802154_AUTOACK_HW
#else
#define NULLRDC_802154_AUTOACK_HW 0
#endif /* NULLRDC_CONF_802154_AUTOACK_HW */
#endif /* NULLRDC_802154_AUTOACK_HW */

#if NULLRDC_802154_AUTOACK
#include "sys/rtimer.h"
#include "dev/watchdog.h"

#ifdef NULLRDC_CONF_ACK_WAIT_TIME
#define ACK_WAIT_TIME NULLRDC_CONF_ACK_WAIT_TIME
#else /* NULLRDC_CONF_ACK_WAIT_TIME */
#define ACK_WAIT_TIME                      RTIMER_SECOND / 2500
#endif /* NULLRDC_CONF_ACK_WAIT_TIME */
#ifdef NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#define AFTER_ACK_DETECTED_WAIT_TIME NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#else /* NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#define AFTER_ACK_DETECTED_WAIT_TIME       RTIMER_SECOND / 1500
#endif /* NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#endif /* NULLRDC_802154_AUTOACK */

#ifdef NULLRDC_CONF_SEND_802154_ACK
#define NULLRDC_SEND_802154_ACK NULLRDC_CONF_SEND_802154_ACK
#else /* NULLRDC_CONF_SEND_802154_ACK */
#define NULLRDC_SEND_802154_ACK 0
#endif /* NULLRDC_CONF_SEND_802154_ACK */

#if NULLRDC_SEND_802154_ACK
#include "net/mac/frame802154.h"
#endif /* NULLRDC_SEND_802154_ACK */

#define ACK_LEN 3
#define ROOT_ADDR 1
// uint8_t WUR_ADDRESS_LENGTH;
// uint8_t WUR_ADDRESS_BUFFER[LINKADDR_SIZE];  // TX and RX buffer for the WUR

// #define SIM_COLLECT_IMI 60000
// #define SIM_SETTLING_TIME 10000

/*---------------------------------------------------------------------------*/
static void
on(void)
{
    NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
    NETSTACK_RADIO.off();
}
/*---------------------------------------------------------------------------*/
static void wur_trigger_tx() {
    wur_set_tx();
    clock_delay(100);
    wur_clear_tx();
}
/*---------------------------------------------------------------------------*/
static int
send_one_packet(mac_callback_t sent, void *ptr)
{
  on();
  int ret;
  int last_sent_ok = 0;
  
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);

#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
  //packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 0);
#endif /* NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW */

  if(NETSTACK_FRAMER.create() < 0) {
    /* Failed to allocate space for headers */
    PRINTF("wurrdc: send failed, too large header\n");
    ret = MAC_TX_ERR_FATAL;
  } else {
#if NULLRDC_802154_AUTOACK
    int is_broadcast;

    NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());

    is_broadcast = packetbuf_holds_broadcast();

    if(NETSTACK_RADIO.receiving_packet() ||
       (!is_broadcast && NETSTACK_RADIO.pending_packet())) {
      ret = MAC_TX_COLLISION;
      off();/*****************************/
    } else {
      if(!is_broadcast) {
        RIMESTATS_ADD(reliabletx);
      }

      switch(NETSTACK_RADIO.transmit(packetbuf_totlen())) {
      case RADIO_TX_OK:
        if(is_broadcast) {
          off();
          ret = MAC_TX_OK;
        } else {
          off(); /*this turns off the radio right after sending the packet without waiting for ACK*/
          ret = MAC_TX_NOACK;
        }
        break;
      case RADIO_TX_COLLISION:
        ret = MAC_TX_COLLISION;
        off();
        break;
      default:
        ret = MAC_TX_ERR;
        off();
        break;
      }
    }
#else /* ! NULLRDC_802154_AUTOACK */

    switch(NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen())) { 
    case RADIO_TX_OK:
      ret = MAC_TX_OK;
      break;
    case RADIO_TX_COLLISION:
      ret = MAC_TX_COLLISION;
      break;
    case RADIO_TX_NOACK:
      ret = MAC_TX_NOACK;
      break;
    default:
      ret = MAC_TX_ERR;
      break;
    }
#endif /* ! NULLRDC_802154_AUTOACK */
  }
  if(ret == MAC_TX_OK) {
    last_sent_ok = 1;
  }
  mac_call_sent_callback(sent, ptr, ret, 1);

  return last_sent_ok;
}
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  send_one_packet(sent, ptr);
}

/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  while(buf_list != NULL) {
    struct rdc_buf_list *next = buf_list->next;
    int last_sent_ok;

    queuebuf_to_packetbuf(buf_list->buf);

    last_sent_ok = send_one_packet(sent, ptr);

    if(!last_sent_ok) {
      return;
    }
    buf_list = next;
  }
}

/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{

#if NULLRDC_SEND_802154_ACK
  int original_datalen;
  uint8_t *original_dataptr;

  original_datalen = packetbuf_datalen();
  original_dataptr = packetbuf_dataptr();
#endif

#if NULLRDC_802154_AUTOACK
  if(packetbuf_datalen() == ACK_LEN) {
  	    /* Ignore ack packets */
    PRINTF("wurrdc: ignored ack\n"); 
  } else
#endif /* NULLRDC_802154_AUTOACK */
  if(NETSTACK_FRAMER.parse() < 0) {
    PRINTF("wurrdc: failed to parse %u\n", packetbuf_datalen());
#if NULLRDC_ADDRESS_FILTER
  } else if(!linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                         &linkaddr_node_addr) &&
            !packetbuf_holds_broadcast()) {
    PRINTF("wurrdc: not for us\n");
#endif /* NULLRDC_ADDRESS_FILTER */
  } else {
    int duplicate = 0;

#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
#if RDC_WITH_DUPLICATE_DETECTION
    /* Check for duplicate packet. */
    duplicate = mac_sequence_is_duplicate();
    if(duplicate) {
      /* Drop the packet. */  
      PRINTF("wurrdc: drop duplicate link layer packet %u\n",
             packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO));
    } else {
      mac_sequence_register_seqno();
    }
#endif /* RDC_WITH_DUPLICATE_DETECTION */
#endif /* NULLRDC_802154_AUTOACK */

    if(!duplicate) {
      on(); 
      NETSTACK_MAC.input();   
    }
  }

}
/*---------------------------------------------------------------------------*/

//PROCESS(wur_broadcast_process, "wur event handler process");

static void
init(void)
{
  printf("wurrdc: Initialized\n");
  //wur_init();
  //process_start(&wur_broadcast_process, NULL);
  on();
}
/*---------------------------------------------------------------------------*/
static int
turn_on(void)
{
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
turn_off(int keep_radio_on)
{
  if(keep_radio_on) {
    return NETSTACK_RADIO.on();
  } else {
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver wrimac_broadcast_driver = {
  "wrimac-broadcast",
  init,
  send_packet,
  send_list,
  packet_input,
  turn_on,
  turn_off,
  channel_check_interval,
};

/*---------------------------------------------------------------------------*/
// PROCESS_THREAD(wur_broadcast_process, ev, data)
// {
//   // static struct etimer timeout;
//   // static struct etimer periodic;
//   // static struct etimer settle_timer;
 
//   PROCESS_BEGIN();

//   etimer_set(&settle_timer, (uint32_t)SIM_SETTLING_TIME * CLOCK_SECOND / 1000);
//   PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&settle_timer));
//   off();

//   if ((node_id == ROOT_ADDR)){
//     while(1) {
//       etimer_set(&periodic, (uint32_t)SIM_COLLECT_IMI * CLOCK_SECOND / 1000);
//       PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic));

//       WUR_ADDRESS_LENGTH = LINKADDR_SIZE;
//       printf("Request Data \n");
//       wur_trigger_tx();

//       on();
//       etimer_set(&timeout, 5); //125ms radio ON window time
//       PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timeout));
//       off();
//     } 
//   } 
//   PROCESS_END();
// }
