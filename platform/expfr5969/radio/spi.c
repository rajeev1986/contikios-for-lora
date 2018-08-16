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
 * spi.c
 *
 * \Contiki port, author 
 *  Rajeev Piyare <rajeev.piyare@hotmail.com>
 *  Created : 2018-02-12
 */
#include "contiki.h"
#include <msp430.h>
#include "spi.h"
#include <stdint.h>

volatile uint8_t spi_buf = 0;

/*
 * This is SPI initialization code for the MSP430X architecture.
 *
 */
//  Mapping MSP430FR5969 <-> SX1276 on ETH LoRa platform (May 2017)
//                   MSP430FR5969
//                 -----------------
//                |                 |
//                |             P2.0|-> Data Out (UCA0SIMO)
//                |                 |
//                |             P2.1|<- Data In (UCA0SOMI)
//                |                 |
//                |             P1.5|-> Serial Clock Out (UCA0CLK)
//                |                 |
//                |             P3.0|-> CS
//                |                 |
//                |             P1.4|-> RESET
//                |                 |
//                |             P1.3|<-> DIO_0
//                |                 |
//                 -----------------
/*
 * Initialize SPI bus.
 */
void spi_init(void)
{
  // Configure GPIOS
  P1DIR |= RESET;
  P1OUT &= RESET;

  P3DIR |= CS;
  P3OUT |= CS;

  P1SEL0 &= ~SCLK;
  P2SEL1 |= (MOSI | MISO); 
  P1SEL1 |= SCLK;
  P2SEL1 |= (MOSI | MISO);

  // Initialize ports for communication with SPI units.

  UCA0CTLW0 = UCSWRST;               // Put state machine in reset
  UCA0CTLW0 |= UCSSEL_2;             // SMCLK
  UCA0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // MSB-first 8-bit, Master, Synchronous, 3 pin SPI master, no ste, watch-out for clock-phase UCCKPH

  UCA0BR0 = 0x02;
  UCA0BR1 = 0x00;

  UCA0CTLW0 &= ~UCSWRST;  // Initializing USCI and Remove RESET before enabling interrupts

}

void dio0irq_init(){

  P1DIR &= ~DIO_0;   // configure as input for the interrupt signal
  P1OUT &= ~DIO_0;   // Pull down
  P1IE  &= ~DIO_0;   // Interrupt disabled
  P1IFG &= ~DIO_0;   // Clear DIO0 interrupt
  P1IES &= ~DIO_0;   // Interrupt on a low to high transition

}

void spi_txready() {
  while (!(UCA0IFG & UCTXIFG)); // TX buffer ready?
}

void spi_rxready() {
  while (!(UCA0IFG & UCRXIFG)); // RX Received?
}

void spi_send(uint8_t data) {
  spi_txready();
  UCA0TXBUF = data;            // Send data over SPI to Slave
}

void spi_recv() {
  spi_rxready();
  spi_buf = UCA0RXBUF;         // Store received data
}

void spi_transfer(uint8_t data) {
  spi_send(data);
  spi_recv();
}

void spi_chipEnable() {
  P3OUT &= ~CS;
}

void spi_chipDisable() {
   P3OUT |= CS;
}

