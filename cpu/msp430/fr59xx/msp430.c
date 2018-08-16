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

/**
 * \file
 *         msp430.c
 * \Contiki port, author 
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 * \Adopted from:
 *         Andrea Gaglione <and.gaglione@gmail.com>
 *         David Rodenas-Herraiz <dr424@cam.ac.uk>
 */

#include "contiki.h"
#include "dev/watchdog.h"
#include "sys/energest.h"
#include "isr_compat.h"

/* dco_required set to 1 will cause the CPU not to go into
   sleep modes where the DCO clock stopped */
int msp430_dco_required;

#if defined(__MSP430__) && defined(__GNUC__)
#define asmv(arg) __asm__ __volatile__(arg)
#endif

/*---------------------------------------------------------------------------*/
ISR(UNMI, unmi_isr)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  watchdog_start();

  CSCTL0_H = CSKEY >> 8;             /* Enable write access to the CS module */
  do
  {
    CSCTL5 &= ~LFXTOFFG;             /* Clear LFXT fault interrupt flag */
    SFRIFG1 &= ~OFIFG;
  }while (SFRIFG1&OFIFG);            /* Test LFXT fault interrupt flag */
  CSCTL0_H = 0;                      /* Lock CS registers */

  watchdog_stop();

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
void
msp430_init_dco(void)
{
  PJSEL0 |= BIT4 | BIT5;            /* LFXT Setup */
  PJSEL1 &= ~(BIT4 | BIT5);

  CSCTL0_H = CSKEY >> 8;            /* Enable write access to the CS module */

#if F_CPU == 8000000uL
  CSCTL1 = DCOFSEL_6;               /*  Set DCO to 8MHz */

#elif F_CPU == 1000000uL
  CSCTL1 = DCOFSEL_0;               /*  Set DCO to 1MHz */
#else

#error Unsupported DCO setting msp430.c

#endif

  CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK; /* Set ACLK = LFXT;
                                                         * SMCLK = MCLK = DCO */
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     /* Set all dividers */
  CSCTL4 |= LFXTDRIVE_0;            /* Lowest drive strength and current
                                     * consumption LFXT oscillator */
  CSCTL4 &= ~LFXTOFF;               /* LFXT is turned on */

  do
  {
    CSCTL5 &= ~LFXTOFFG;            /* Clear LFXT fault interrupt flag */
    SFRIFG1 &= ~OFIFG;
  } while (SFRIFG1 & OFIFG);        /* Test LFXT fault interrupt flag */

  SFRIE1 |= OFIE;                   /* Enable fault interrupt */

  CSCTL0_H = 0;                     /* Lock CS registers */
}
/*---------------------------------------------------------------------------*/
static void
init_ports(void)
{
#ifdef REFCTL0
  REFCTL0 |= REFTCOFF;
  REFCTL0 &= ~REFON;
#endif

#ifdef P1SEL0
    P1SEL0 = 0;
#endif
#ifdef P1SEL1
    P1SEL1 = 0;
#endif
#ifdef P2SEL0
    P2SEL0 = 0;
#endif
#ifdef P2SEL1
    P2SEL1 = 0;
#endif
#ifdef P3SEL1
    P3SEL1 = 0;
#endif
#ifdef P4SEL0
    P4SEL0 = 0;
#endif
#ifdef P4SEL1
    P4SEL1 = 0;
#endif

  PJSEL0 = 0;
  PJSEL1 = 0;

#ifdef P1DIR
  P1OUT = 0;
  P1DIR = 0xFF;
  P1REN = 0xFF;
#endif
#ifdef P2DIR
  P2OUT = 0;
  P2DIR = 0xFF;
  P2REN = 0xFF;
#endif
#ifdef P3DIR
  P3OUT = 0;
  P3DIR = 0xFF;
  P3REN = 0xFF;
#endif
#ifdef P4DIR
  P4OUT = 0;
  P4DIR = 0xFF;
  P4REN = 0xFF;
#endif

  PJOUT = 0;
  PJDIR = 0xFFFF;
  PJREN = 0xFF;

  P1IE = 0;
  P2IE = 0;
  P3IE = 0;
  P4IE = 0;

}
/*--------------------------------------------------------------------------*/
static void
unlock_PMM_module(void)
{
  // Disable the GPIO power-on default high-impedance mode to activate
  // previously configured port settings
  PM5CTL0 &= ~LOCKLPM5;
}
/*---------------------------------------------------------------------------*/
void
msp430_enter_LPMx_5(unsigned char LPM_bits)
{
  init_ports();

  /* Enable interrupt from LFXT to wake up from LPMx.5 */
  PJSEL0 |= BIT4 + BIT5;
  PJSEL1 &= ~(BIT4 + BIT5);

  PMMCTL0_H = PMMPW_H;    /* Unlock PMMPW register */
  PMMCTL0_L |= PMMREGOFF; /* Set PMMREGOFF -> LDO turned off on LPMx.5 entry */
  PMMCTL0_L &= ~(SVSHE);  /* Disable SVS */
  _BIS_SR(LPM_bits);      /* Enter LPMx.5 */
}
/*---------------------------------------------------------------------------*/
/* msp430-ld may align _end incorrectly. Workaround in cpu_init. */
#if defined(__MSP430__) && defined(__GNUC__)
extern int _end;    /* Not in sys/unistd.h */
static char *cur_break = (char *)&_end;
#endif
/*---------------------------------------------------------------------------*/
/* add/remove_lpm_req - for requiring a specific LPM mode. currently Contiki */
/* jumps to LPM3 to save power, but DMA will not work if DCO is not clocked  */
/* so some modules might need to enter their LPM requirements                */
/* NOTE: currently only works with LPM1 (e.g. DCO) requirements.             */
/*---------------------------------------------------------------------------*/
void
msp430_add_lpm_req(int req)
{
  if(req <= MSP430_REQUIRE_LPM1) {
    msp430_dco_required++;
  }
}

void
msp430_remove_lpm_req(int req)
{
  if(req <= MSP430_REQUIRE_LPM1) {
    msp430_dco_required--;
  }
}

void
msp430_cpu_init(void)
{
  dint();
  unlock_PMM_module();
  watchdog_init();
  //init_ports();   /* Disabling This reduces the power cons on the current board...TODO */
  msp430_init_dco();
  eint();
#if defined(__MSP430__) && defined(__GNUC__)
  if((uintptr_t)cur_break & 1) { /* Workaround for msp430-ld bug! */
    cur_break++;
  }
#endif

  msp430_dco_required = 0;
}
/*---------------------------------------------------------------------------*/

#define STACK_EXTRA 32
/*
 * Allocate memory from the heap. Check that we don't collide with the
 * stack right now (some other routine might later). A watchdog might
 * be used to check if cur_break and the stack pointer meet during
 * runtime.
 */
#if defined(__MSP430__) && defined(__GNUC__)
void *
sbrk(int incr)
{
  char *stack_pointer;

  asmv("mov r1, %0" : "=r" (stack_pointer));
  stack_pointer -= STACK_EXTRA;
  if(incr > (stack_pointer - cur_break))
    return (void *)-1;    /* ENOMEM */

  void *old_break = cur_break;
  cur_break += incr;
  /*
   * If the stack was never here then [old_break .. cur_break] should
   * be filled with zeros.
  */
  return old_break;
}
#endif
/*---------------------------------------------------------------------------*/
/*
 * Mask all interrupts that can be masked.
 */
int
splhigh_(void)
{
  int sr;
  /* Clear the GIE (General Interrupt Enable) flag. */
#ifdef __IAR_SYSTEMS_ICC__
  sr = __get_SR_register();
  __bic_SR_register(GIE);
#else
  asmv("mov r2, %0" : "=r" (sr));
  asmv("bic %0, r2" : : "i" (GIE));
#endif
  return sr & GIE;    /* Ignore other sr bits. */
}
/*---------------------------------------------------------------------------*/
#ifdef __IAR_SYSTEMS_ICC__
int __low_level_init(void)
{
  /* turn off watchdog so that C-init will run */
  WDTCTL = WDTPW + WDTHOLD;
  /*
   * Return value:
   *
   *  1 - Perform data segment initialization.
   *  0 - Skip data segment initialization.
   */
  return 1;
}
#endif
/*---------------------------------------------------------------------------*/
