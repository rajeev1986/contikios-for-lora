/*
 * Copyright (c) 2011, Swedish Institute of Computer Science
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/*
 * \Contiki port, author 
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 */

/*
 * Machine dependent MSP430X UART1 code.
 */

#include "contiki.h"
#include <stdlib.h>
#include "sys/energest.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "isr_compat.h"

static int (*uart1_input_handler)(unsigned char c);

static volatile uint8_t transmitting;

/*---------------------------------------------------------------------------*/
uint8_t
uart1_active(void)
{
  return (UCA1STATW & UCBUSY) | transmitting;
}
/*---------------------------------------------------------------------------*/
void
uart1_set_input(int (*input)(unsigned char c))
{

  uart1_input_handler = input;
}
/*---------------------------------------------------------------------------*/
void
uart1_writeb(unsigned char c)
{
  watchdog_periodic();
  /* Loop until the transmission buffer is available. */
  while((UCA1STATW & UCBUSY));

  /* Transmit the data. */
  UCA1TXBUF = c;
}
/*---------------------------------------------------------------------------*/
/**
 * Initalize the RS232 port.
 *
 */
void
uart1_init(unsigned long ubr)
{
  /* RS232 */
  UCA1CTL1 |= UCSWRST;            /* Hold peripheral in reset state */
  UCA1CTL1 |= UCSSEL__SMCLK;      /* CLK = SMCLK */

#if F_CPU == 8000000uL
  // Baud Rate calculation
  UCA1BR0 = 0x45;                 /* 8MHz/115200 = 69 = 0x45 */
  UCA1BR1 = 0x00;
  UCA1MCTLW = 0x5500;             /* Modulation: 8000000/115200 - INT(8000000/115200)=0.44*/

#elif F_CPU == 1000000uL
  UCA1BR0 = 6;                    /* 1MHz/9600*/
  UCA1BR1 = 0;
  UCA1MCTLW |= UCOS16 + UCBRF_8;

#else

#error Unsupported CPU speed in uart1.c

#endif

#if 0
  ubr = (MSP430_CPU_SPEED / ubr); // It's not computing this correctly...
  UCA1BR0 = ubr & 0xff;
  UCA1BR1 = (ubr >> 8) & 0xff;
  UCA1MCTLW = UCBRS3;             /* Modulation UCBRSx = 3 */
#endif

  /* P2.5,6 = EUSCI_A1 TXD/RXD */

  P2DIR &= ~BIT6;
  P2DIR |= BIT5;
  P2SEL1 |= BIT5 + BIT6;
  P2SEL0 &= ~(BIT5 + BIT6);

  UCA1CTL1 &= ~UCSWRST;           /* Initialize USCI state machine */

  transmitting = 0;

  /* XXX Clear pending interrupts before enable */
  UCA1IE &= ~UCRXIFG;
  UCA1IE &= ~UCTXIFG;

  UCA1CTL1 &= ~UCSWRST;                   /* Initialize USCI state machine **before** enabling interrupts */
  UCA1IE |= UCRXIE;                        /* Enable UCA1 RX interrupt */

}
/*---------------------------------------------------------------------------*/

ISR(USCI_A1, uart1_rx_interrupt)
{
  uint8_t c;

  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  if(UCA1IV == 2) {
    if(UCA1STATW & UCRXERR) {
      c = UCA1RXBUF;   /* Clear error flags by forcing a dummy read. */
    } else {
      c = UCA1RXBUF;
      if(uart1_input_handler != NULL) {
        if(uart1_input_handler(c)) {
          LPM4_EXIT;
        }
      }
    }
  }
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

/*---------------------------------------------------------------------------*/
