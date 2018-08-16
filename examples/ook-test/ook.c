/*
 *  OOK transmitter.c
 *  Simple wake-up radio packet transmitter example in Contiki using OOK Modulation
 *  Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Created : 2018-02-28
 */                                                            

#include "contiki.h"
#include "dev/leds.h"
#include <stdio.h>
#include "sx1276.h"
#include "sx1276-arch.h"
#include "spi.h"
/*---------------------------------------------------------------------------*/
#define LEDS_DEBUG 0
#if LEDS_DEBUG
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#define LEDS_TOGGLE(x) leds_toggle(x)
#else
#define LEDS_ON(x)
#define LEDS_OFF(x)
#define LEDS_TOGGLE(x)
#endif
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif
/*---------------------------------------------------------------------------*/
static struct etimer tx_timer;

uint8_t wub[] = {0xA5};
#define BUFFER_SIZE   2 // Define the payload size here
uint8_t buffer[BUFFER_SIZE];

void SendPing() {
  buffer[0] = 0x5A;

  sx1276_send(buffer, 1);
}

/*---------------------------------------------------------------------------*/
PROCESS(rx_process, "TX process");
AUTOSTART_PROCESSES(&rx_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rx_process, ev, data)
{
  PROCESS_BEGIN();

	sx1276_driver.init();
	sx1276_driver.on();

	etimer_set(&tx_timer, 5*CLOCK_SECOND);

  while( 1 )
  {
    PROCESS_WAIT_EVENT();

    if(ev == PROCESS_EVENT_TIMER) {
    	leds_on(LEDS_ALL);
      sx1276_send((uint8_t*)wub, 1);
    	printf("Sent the OOK Beacon for WuRX \n");
    	leds_off(LEDS_ALL);

		etimer_reset(&tx_timer);
    }
  }
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
