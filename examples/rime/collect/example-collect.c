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
 * \file
 *         Example of how the collect primitive works over LoRa platform
 * \author
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "net/rime/collect.h"
#include "net/netstack.h"
#include "node-id.h"
#include "sx1276-arch.h"
#include <stdio.h>

static struct collect_conn tc;
#define SEND_INTERVAL 20
#define COLLECT_MAX_RETRANSMITS 2
/*---------------------------------------------------------------------------*/
PROCESS(example_collect_process, "Test collect process");
AUTOSTART_PROCESSES(&example_collect_process);
/*---------------------------------------------------------------------------*/
static void
recv(const linkaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  printf("Sink got message from %d.%d, seqno %d, hops %d: len %d '%s'\n",
	 originator->u8[0], originator->u8[1],
	 seqno, hops,
	 packetbuf_datalen(),
	 (char *)packetbuf_dataptr());
}
/*---------------------------------------------------------------------------*/
static const struct collect_callbacks callbacks = { recv };
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_collect_process, ev, data)
{
  static struct etimer periodic;
  static struct etimer et;
  
  PROCESS_BEGIN();
  sx1276_driver.init(); 

  collect_open(&tc, 130, COLLECT_ROUTER, &callbacks);

  if(linkaddr_node_addr.u8[0] == 1 && linkaddr_node_addr.u8[1] == 0) {
    	printf("I am sink\n");
    	collect_set_sink(&tc, 1);
      /* Keep listening at the sink for incoming data */
      NETSTACK_RADIO.on();
  }

  /* Allow some time for the CTP network to settle */
  etimer_set(&et, 120 * CLOCK_SECOND);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));
 
  if (node_id != 1) { /* If I am not Sink */
    while(1) {
      etimer_set(&et, CLOCK_SECOND * SEND_INTERVAL);

      PROCESS_WAIT_EVENT();

      if(etimer_expired(&et)) {
          printf("Sending the packet\n");
          packetbuf_clear();
          packetbuf_set_datalen(sprintf(packetbuf_dataptr(), "%s", "Hello") + 1);
          collect_send(&tc, COLLECT_MAX_RETRANSMITS);
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
