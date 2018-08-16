
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
 *         Best-effort single-hop unicast example over LoRa with Sequence number tracing
 * \author
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
  * \date
 *         2018-03-14
 */
#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "net/netstack.h"
#include "dev/leds.h"
#include <stdio.h>
#include "node-id.h"
#include "sx1276.h"
#include "sx1276-arch.h"
#include "spi.h"

#define SIM_PASSWORD 	0xEFBEADDE
#define ROOT_ADDR 		1
#define SEND_INTERVAL	10

static struct simmsg sim_msg;

struct simmsg {
  uint16_t seqn;
  uint32_t password; 
};

/*---------------------------------------------------------------------------*/
PROCESS(example_unicast_process, "Unicast Communication");
AUTOSTART_PROCESSES(&example_unicast_process);
/*---------------------------------------------------------------------------*/
void callback_recv(uint16_t from, uint8_t* payload, uint8_t length)
{
  struct simmsg msg;
  memcpy(&msg, payload, sizeof(struct simmsg));
  printf("Recv from %u seqn %u\n", from, msg.seqn);
}
/*---------------------------------------------------------------------------*/
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{ 
  callback_recv(*(uint16_t*)from, packetbuf_dataptr(), packetbuf_datalen());
}
/*---------------------------------------------------------------------------*/
static void
sent_uc(struct unicast_conn *c, int status, int num_tx)
{
  const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  if(linkaddr_cmp(dest, &linkaddr_null)) {
    return;
  }
}
/*---------------------------------------------------------------------------*/
static const struct unicast_callbacks unicast_callbacks = {recv_uc, sent_uc};
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_unicast_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);)
    
  PROCESS_BEGIN();
  static struct etimer et;
  sx1276_driver.init(); 

  sim_msg.seqn = 0;
  sim_msg.password = SIM_PASSWORD;
  unicast_open(&uc, 146, &unicast_callbacks);

  while(1) {
    linkaddr_t addr;
    etimer_set(&et, CLOCK_SECOND * SEND_INTERVAL);

    NETSTACK_RADIO.on();

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  	packetbuf_copyfrom(&sim_msg, sizeof(sim_msg));
  	addr.u8[0] = 1;
  	addr.u8[1] = 0;
  	if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
      /* If node ID is not 1, send the data packet to sink */
    		printf("Send to %u seqn %u\n", ROOT_ADDR, sim_msg.seqn);
    		unicast_send(&uc, &addr);
    		sim_msg.seqn ++;
  }
}
	  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
