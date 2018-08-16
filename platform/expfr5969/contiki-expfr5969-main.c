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
 *
 * \Contiki port, author 
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 */

#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"
#include "spi.h"
#include "sx1276.h"
#include "sx1276-arch.h"

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /* NETSTACK_CONF_WITH_IPV6 */

#include "net/rime/rime.h"

#include "sys/node-id.h"
#include "sys/autostart.h"

extern unsigned char node_mac[8];

#if UIP_CONF_ROUTER

#ifndef UIP_ROUTER_MODULE
#ifdef UIP_CONF_ROUTER_MODULE
#define UIP_ROUTER_MODULE UIP_CONF_ROUTER_MODULE
#else /* UIP_CONF_ROUTER_MODULE */
#define UIP_ROUTER_MODULE rimeroute
#endif /* UIP_CONF_ROUTER_MODULE */
#endif /* UIP_ROUTER_MODULE */

extern const struct uip_router UIP_ROUTER_MODULE;
#endif /* UIP_CONF_ROUTER */

#ifndef NETSTACK_CONF_WITH_IPV4
#define NETSTACK_CONF_WITH_IPV4 0
#endif
#if NETSTACK_CONF_WITH_IPV4
#include "net/ip/uip.h"
#include "net/ipv4/uip-fw.h"
#include "net/uip-fw-drv.h"
#include "net/ipv4/uip-over-mesh.h"
static struct uip_fw_netif slipif =
{ UIP_FW_NETIF(192, 168, 1, 2, 255, 255, 255, 255, slip_send) };
static struct uip_fw_netif meshif =
{ UIP_FW_NETIF(172, 16, 0, 0, 255, 255, 0, 0, uip_over_mesh_send) };

#endif /* NETSTACK_CONF_WITH_IPV4 */

#define UIP_OVER_MESH_CHANNEL 8
#if NETSTACK_CONF_WITH_IPV4
static uint8_t is_gateway;
#endif /* NETSTACK_CONF_WITH_IPV4 */

#ifdef EXPERIMENT_SETUP
#include "experiment-setup.h"
#endif

#define DEBUG_LEDS 1
#if DEBUG_LEDS
#define LEDS_INIT() leds_init()
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#define LEDS_TOGGLE(x) leds_toggle(x)
#else
#define LEDS_INIT()
#define LEDS_ON(x)
#define LEDS_OFF(x)
#define LEDS_TOGGLE(x)
#endif

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif

#define WITH_SERIAL_COMM 1

void init_platform(void);

/*---------------------------------------------------------------------------*/
#ifndef RF_CHANNEL
#define RF_CHANNEL              26
#endif
/*---------------------------------------------------------------------------*/
void uip_log(char *msg) { puts(msg); }
/*---------------------------------------------------------------------------*/
#ifndef NODE_ID
#define NODE_ID 0x01
#endif /* NODE_ID */
static void
set_rime_addr(void)
{
  linkaddr_t n_addr;
  int i;

  memset(&n_addr, 0, sizeof(linkaddr_t));

  //  Set node address
#if NETSTACK_CONF_WITH_IPV6
  n_addr.u8[7] = node_id & 0xff;
  n_addr.u8[6] = node_id >> 8;
#else
  n_addr.u8[0] = node_id & 0xff;
  n_addr.u8[1] = node_id >> 8;
#endif

  linkaddr_set_node_addr(&n_addr);

  PRINTF("Rime started with address ");
  for(i = 0; i < sizeof(n_addr.u8) - 1; i++) {
    PRINTF("%d.", n_addr.u8[i]);
  }
  PRINTF("%d\n", n_addr.u8[i]);

}
/*---------------------------------------------------------------------------*/
#if !PROCESS_CONF_NO_PROCESS_NAMES
static void
print_processes(struct process * const processes[])
{
  /*  const struct process * const * p = processes;*/
  printf("Starting");
  while(*processes != NULL) {
    printf(" '%s'", (*processes)->name);
    processes++;
  }
  putchar('\n');
}
#endif /* !PROCESS_CONF_NO_PROCESS_NAMES */
/*--------------------------------------------------------------------------*/
#if NETSTACK_CONF_WITH_IPV4
static void
set_gateway(void)
{
  if(!is_gateway) {
    LEDS_ON(LEDS_RED);
    /*
    printf("%d.%d: making myself the IP network gateway.\n\n",
    linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
    printf("IPv4 address of the gateway: %d.%d.%d.%d\n\n",
    uip_ipaddr_to_quad(&uip_hostaddr));
    */
    uip_over_mesh_set_gateway(&linkaddr_node_addr);
    uip_over_mesh_make_announced_gateway();
    is_gateway = 1;
  }
}
#endif /* NETSTACK_CONF_WITH_IPV4 */
/*---------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
  /*
   * Re-initialize hardware
   */
  msp430_cpu_init();

  clock_init();
  LEDS_INIT();
  spi_init();

  LEDS_ON(LEDS_RED);

  clock_wait(2);
  process_init();

#if WITH_SERIAL_COMM
  uart1_init(115200); /* Must come before first printf */
#endif

//#if NETSTACK_CONF_WITH_IPV4
//  slip_arch_init(115200);
//#endif /* NETSTACK_CONF_WITH_IPV4 */

  clock_wait(2);

  LEDS_ON(LEDS_GREEN);

  LEDS_OFF(LEDS_RED);

  rtimer_init();

#if WITH_SERIAL_COMM
  uart1_set_input(serial_line_input_byte);
  serial_line_init();
#endif

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  /*
   * Hardware initialization done!
   */

  /* Restore node id if such has been stored in information memory */
  node_id_restore();

#ifdef NODE_ID
  if(!node_id) {
    node_id = NODE_ID;
  }
#endif

  /* If no MAC address was burned, we use the node id */
  if(!(node_mac[0] | node_mac[1] | node_mac[2] | node_mac[3] |
       node_mac[4] | node_mac[5] | node_mac[6] | node_mac[7])) {

    node_mac[0] = 0xDA;  /* Hardcoded for EXP-MSP430FR5969 */
    node_mac[1] = 0x0A;  /* Hardcoded for Revision A */
    node_mac[2] = 0x00;  /* Hardcoded to arbitrary even number so that
                            the 802.15.4 MAC address is compatible with
                            an Ethernet MAC address - byte 0 (byte 2 in
                            the DS ID) */
    node_mac[3] = 0x00;  /* Hardcoded */
    node_mac[4] = 0x00;  /* Hardcoded */
    node_mac[5] = 0x00;  /* Hardcoded */
    node_mac[6] = node_id >> 8;
    node_mac[7] = node_id & 0xff;
  }

  /* Overwrite node MAC if desired at compile time */
#ifdef MAC_ID
#warning "***** CHANGING DEFAULT MAC *****"
  node_mac[0] = 0xDA; /* Hardcoded for EXP-MSP430FR5969 */
  node_mac[1] = 0x0A; /* Hardcoded for Revision A */
  node_mac[3] = 0x00; /* Hardcoded to arbitrary even number so that
                         the 802.15.4 MAC address is compatible with
                         an Ethernet MAC address - byte 0 (byte 2 in
                         the DS ID) */
  node_mac[3] = 0x00; /* Hardcoded */
  node_mac[4] = 0x00; /* Hardcoded */
  node_mac[5] = 0x00; /* Hardcoded */
  node_mac[6] = MAC_ID >> 8;
  node_mac[7] = MAC_ID & 0xff;
#endif

#ifdef IEEE_802154_MAC_ADDRESS
  /* for setting "hardcoded" IEEE 802.15.4 MAC addresses */
  {
    uint8_t ieee[] = IEEE_802154_MAC_ADDRESS;
    memcpy(node_mac, ieee, sizeof(uip_lladdr.addr));
    node_mac[7] = node_id & 0xff;
  }
#endif

  /*
   * Initialize Contiki and our processes.
   */
  process_start(&etimer_process, NULL);

  ctimer_init();

  init_platform();

  set_rime_addr();
  random_init(linkaddr_node_addr.u8[LINKADDR_SIZE-2] + linkaddr_node_addr.u8[LINKADDR_SIZE-1]);

  {
    uint8_t longaddr[8];
    // uint16_t shortaddr;

    // shortaddr = (linkaddr_node_addr.u8[0] << 8) +
    //   linkaddr_node_addr.u8[1];
    memset(longaddr, 0, sizeof(longaddr));
    linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);

    PRINTF("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
           longaddr[0], longaddr[1], longaddr[2], longaddr[3],
           longaddr[4], longaddr[5], longaddr[6], longaddr[7]);
  }
  /* Print the Node ID */
  if(node_id > 0) {
    PRINTF("Node id is set to %u.\n", node_id);
  } else {
    PRINTF("Node id is not set.\n");
  }

#if NETSTACK_CONF_WITH_IPV6

  queuebuf_init();
  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_LLSEC.init();
  NETSTACK_NETWORK.init();

  PRINTF("%s %s %s\n",
         NETSTACK_LLSEC.name, NETSTACK_MAC.name, NETSTACK_RDC.name);

  process_start(&tcpip_process, NULL);

#if DEBUG
  PRINTF("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      PRINTF("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    PRINTF("%02x%02x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
  }
#endif /* DEBUG */

  if(!UIP_CONF_IPV6_RPL) {
    uip_ipaddr_t ipaddr;
    int i;
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    PRINTF("Tentative global IPv6 address ");
    for(i = 0; i < 7; ++i) {
      PRINTF("%02x%02x:",
             ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
    }
    PRINTF("%02x%02x\n",
           ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
  }
#else /* NETSTACK_CONF_WITH_IPV6 */

  queuebuf_init();
  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_LLSEC.init();
  NETSTACK_NETWORK.init();

  PRINTF("MAC Driver: %s RDC Driver: %s \n",
        NETSTACK_MAC.name, NETSTACK_RDC.name);

#endif /* NETSTACK_CONF_WITH_IPV6 */

  LEDS_OFF(LEDS_GREEN);

#if TIMESYNCH_CONF_ENABLED
  timesynch_init();
  timesynch_set_authority_level((linkaddr_node_addr.u8[0] << 4) + 16);
#endif /* TIMESYNCH_CONF_ENABLED */

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  watchdog_start();
  /* Stop the watchdog */
  watchdog_stop();

#if !PROCESS_CONF_NO_PROCESS_NAMES
  print_processes(autostart_processes);
#else /* !PROCESS_CONF_NO_PROCESS_NAMES */
  putchar('\n'); /* include putchar() */
#endif /* !PROCESS_CONF_NO_PROCESS_NAMES */
  autostart_start(autostart_processes);

  /*
   * This is the scheduler loop.
   */
  while(1) {
    int r;
    do {
      /* Reset watchdog. */
      watchdog_periodic();
      r = process_run();
    } while(r > 0);

    /*
     * Idle processing.
     */
    int s = splhigh();    /* Disable interrupts. */

    if(process_nevents() != 0) {
      splx(s);                  /* Re-enable interrupts. */
    }
    else {
      static unsigned long irq_energest = 0;
      /* Re-enable interrupts and go to sleep atomically. */
      ENERGEST_SWITCH(ENERGEST_TYPE_CPU, ENERGEST_TYPE_LPM);
      /* We only want to measure the processing done in IRQs when we
         are asleep, so we discard the processing time done when we
         were awake. 
      */
      energest_type_set(ENERGEST_TYPE_IRQ, irq_energest);
      watchdog_stop();
      _BIS_SR(GIE | SCG0 | SCG1 | CPUOFF | OSCOFF); 
      /* LPM4 sleep. This statement will block until the CPU is
         woken up by an interrupt that sets the wake up flag. 
      */
      /* We get the current processing time for interrupts that was
         done during the LPM and store it for next time around. 
      */
      dint();
      irq_energest = energest_type_time(ENERGEST_TYPE_IRQ);
      eint();
      watchdog_start();
      ENERGEST_SWITCH(ENERGEST_TYPE_LPM, ENERGEST_TYPE_CPU);
    }
  }
}
/*---------------------------------------------------------------------------*/
#if LOG_CONF_ENABLED
void
log_message(char *m1, char *m2)
{
  printf("%s%s\n", m1, m2);
}
#endif /* LOG_CONF_ENABLED */
