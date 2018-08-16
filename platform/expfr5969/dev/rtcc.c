/*
 * Copyright (c) 2017, David Rodenas-Herraiz
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
 *         rtcc.c
 * \author
 *         David Rodenas-Herraiz <dr424@cam.ac.uk>
 *
 * \brief
 *         Driver for real-time clock RTC_B module in MSP430FR5969
 *
 * \note   This file is adapted from the RE-Mote on-board ultra-low power RTCC
 *         driver
 */

#include "contiki.h"
#include "sys/energest.h"
#include "isr_compat.h"
#include "dev/watchdog.h"
#include "dev/rtcc.h"
#include "dev/leds.h"

#include <string.h>

#include <stdio.h>
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#define RTCA_AE 0x80

/*---------------------------------------------------------------------------*/
/* Callback pointers when interrupt occurs */
void (*rtcc_timer_callback)(uint8_t value);
void (*rtcc_alarm_callback)(uint8_t value);
void (*rtcc_clk_ready_callback)(uint8_t value);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
ISR(RTC, rtcb0)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  watchdog_start();

  switch(RTCIV)
  {
    case RTCIV_NONE:                    // No interrupts
        PRINTF("RTCIV_NONE (none) interrupt\n");
        break;
    case RTCIV_RTCOFIFG:                // RTC oscillator fault
        PRINTF("RTCIV_RT1PSIFG (osc fault) interrupt\n");
        break;
    case RTCIV_RTCRDYIFG:               // RTC ready
        if((rtcc_clk_ready_callback != NULL)) {
          PRINTF("RTCIV_RTCRDYIFG (ready) interrupt\n");
          rtcc_clk_ready_callback(0);
        }
        break;
    case RTCIV_RTCTEVIFG:               // RTC interval timer
        if((rtcc_timer_callback != NULL)) {
          PRINTF("RTCIV_RTCTEVIFG (timer) interrupt\n");
          rtcc_timer_callback(0);
        }
        break;
    case RTCIV_RTCAIFG:                 // RTC user alarm
        if((rtcc_alarm_callback != NULL)) {
          PRINTF("RTCIV_RTCAIFG (alarm) interrupt\n");
          rtcc_alarm_callback(0);
        }
        break;
    case RTCIV_RT0PSIFG:                // RTC pre-scaler 0
        PRINTF("RTCIV_RT0PSIFG (pre-scaler 0) interrupt\n");
        break;
    case RTCIV_RT1PSIFG:                // RTC pre-scaler 1
        PRINTF("RTCIV_RT1PSIFG (pre-scaler 1) interrupt\n");
        break;
    default: break;
  }

  watchdog_stop();

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
static uint8_t
bcd_to_dec(uint8_t val)
{
  return (uint8_t)(((val >> 4) * 10) + (val % 16));
}
/*---------------------------------------------------------------------------*/
static uint8_t
dec_to_bcd(uint8_t val)
{
  return (uint8_t)(((val / 10) << 4) + (val % 10));
}
/*---------------------------------------------------------------------------*/
static uint8_t
check_leap_year(uint8_t val)
{
  return ((val % 4) && (val % 100)) || (val % 400);
}
/*---------------------------------------------------------------------------*/
static int8_t
rtcc_check_td_format(simple_td_map *data)
{
  /* Using fixed values as these are self-indicative of the variable */
  if((data->seconds > 59) || (data->minutes > 59) || (data->hours > 23)) {
    return RTCC_ERROR;
  }

  if((data->months > 12) || (data->weekdays > 7) || (data->day > 31)) {
    return RTCC_ERROR;
  }

  /* Fixed condition for February (month 2) */
  if(data->months == 2) {
    if(check_leap_year(data->years)) {
      if(data->day > 29) {
        return RTCC_ERROR;
      }
    } else {
      if(data->day > 28) {
        return RTCC_ERROR;
      }
    }
  }
  return RTCC_SUCCESS;
}
/*---------------------------------------------------------------------------*/
int8_t
rtcc_set_time_date(simple_td_map *data)
{
  uint8_t rtc_buffer[RTCC_TD_MAP_SIZE];

  if(rtcc_check_td_format(data) == RTCC_ERROR) {
    PRINTF("RTC: Invalid time/date values\n");
    return RTCC_ERROR;
  }

  rtc_buffer[WEEKDAYLS_ADDR] = dec_to_bcd(data->weekdays);
  rtc_buffer[YEAR_ADDR] = dec_to_bcd(data->years);
  rtc_buffer[MONTHS_ADDR] = dec_to_bcd(data->months);
  rtc_buffer[DAY_ADDR] = dec_to_bcd(data->day);
  rtc_buffer[HOUR_ADDR] = dec_to_bcd(data->hours);
  rtc_buffer[MIN_ADDR] = dec_to_bcd(data->minutes);
  rtc_buffer[SEC_ADDR] = dec_to_bcd(data->seconds);

  RTCCTL01 |= RTCHOLD;  // RTC hold

  RTCSEC =  rtc_buffer[SEC_ADDR];         /* Set seconds */
  RTCMIN =  rtc_buffer[MIN_ADDR];         /* Set minutes */
  RTCHOUR = rtc_buffer[HOUR_ADDR];        /* Set hours */

  RTCDOW =  rtc_buffer[WEEKDAYLS_ADDR];   /* Set day of week */
  RTCDAY =  rtc_buffer[DAY_ADDR];         /* Set day */
  RTCMON =  rtc_buffer[MONTHS_ADDR];      /* Set month */
  RTCYEAR = rtc_buffer[YEAR_ADDR];        /* Set year */

  /* Lock the RTCC and begin count */
  RTCCTL01 &= ~RTCHOLD;

  return RTCC_SUCCESS;
}
/*---------------------------------------------------------------------------*/
int8_t
rtcc_get_time_date(simple_td_map *data)
{
  uint8_t rtc_buffer[RTCC_TD_MAP_SIZE];

  rtc_buffer[SEC_ADDR] = RTCSEC;
  rtc_buffer[MIN_ADDR] = RTCMIN;
  rtc_buffer[HOUR_ADDR] = RTCHOUR;
  rtc_buffer[WEEKDAYLS_ADDR] = RTCDOW;
  rtc_buffer[DAY_ADDR] = RTCDAY;
  rtc_buffer[MONTHS_ADDR] = RTCMON;
  rtc_buffer[YEAR_ADDR] = RTCYEAR;

  data->weekdays = bcd_to_dec(rtc_buffer[WEEKDAYLS_ADDR]);
  data->years = bcd_to_dec(rtc_buffer[YEAR_ADDR]);
  data->months = bcd_to_dec(rtc_buffer[MONTHS_ADDR]);
  data->day = bcd_to_dec(rtc_buffer[DAY_ADDR]);
  data->hours = bcd_to_dec(rtc_buffer[HOUR_ADDR]);
  data->minutes = bcd_to_dec(rtc_buffer[MIN_ADDR]);
  data->seconds = bcd_to_dec(rtc_buffer[SEC_ADDR]);

  return RTCC_SUCCESS;
}
/*---------------------------------------------------------------------------*/
int8_t
rtcc_set_alarm_time_date(simple_td_map *data)
{
  uint8_t buf[RTCC_ALARM_MAP_SIZE];

  if((data == NULL) || (rtcc_check_td_format(data) == RTCC_ERROR)) {
    PRINTF("RTC: invalid alarm values\n");
    return RTCC_ERROR;
  }

  RTCCTL01 |= RTCHOLD;

  /* Clear all alarm registers including the AE bits
   * to prevent potential erroneous alarm conditions
   * from occurring */
  RTCCTL01 &= ~(RTCAIE + RTCAIFG);
  RTCADOW &= RTCA_AE;
  RTCADAY &= RTCA_AE;
  RTCAHOUR &= RTCA_AE;
  RTCAMIN &= RTCA_AE;

  buf[WEEKDAYS_ALARM_ADDR] = dec_to_bcd(data->weekdays);
  buf[DAY_ALARMS_ADDR] = dec_to_bcd(data->day);
  buf[HOURS_ALARM_ADDR] = dec_to_bcd(data->hours);
  buf[MINUTES_ALARM_ADDR] = dec_to_bcd(data->minutes);

  /*
   * By setting the AE bits of RTCADAY, RTCAMIN, RTCAHOUR and RTCADOWDAY,
   * the RTC alarm is enabled.
   */

  RTCADOW = RTCA_AE | (buf[WEEKDAYS_ALARM_ADDR] & 0x7F); /* RTC Day of week alarm */
  RTCADAY = RTCA_AE | (buf[DAY_ALARMS_ADDR] & 0x7F);        /* RTC Day Alarm */
  RTCAHOUR = RTCA_AE | (buf[HOURS_ALARM_ADDR] & 0x7F);      /* RTC Hour Alarm */
  RTCAMIN = RTCA_AE | (buf[MINUTES_ALARM_ADDR] & 0x7F);     /* RTC Minute Alarm */

  // Enable RTC alarm ready interrupt
  RTCCTL01 |= RTCAIE;
  /* Lock the RTCC and begin count */
  RTCCTL01 &= ~RTCHOLD;

  return RTCC_SUCCESS;
}
/*---------------------------------------------------------------------------*/
int8_t
rtcc_init_timer(uint8_t RTCTEV_bits)
{
  switch(RTCTEV_bits){
    case 0x00: break;   // Interrupt at minute changed
    case 0x01: break;   // Interrupt at hour changed
    case 0x02: break;   // Interrupt every day at midnight (00:00)
    case 0x03: break;   // Interrupt every day at noon (12:00)
    default:
        PRINTF("RTC: invalid RTCEV value\n");
        return RTCC_ERROR;
        break;
  }

  RTCCTL01 |= RTCTEVIE | RTCTEV_bits | RTCBCD | RTCHOLD;  /* RTC enable, BCD mode, RTC hold
                                               * enable RTC read ready interrupt
                                               * enable RTC time ready interrupt (1 min)
                                               */
#if 0
  /* Init time */
  RTCSEC =  0;        /* Seconds */
  RTCMIN =  0;        /* Minutes */
  RTCHOUR = 0x03;        /* Hour = 0x03 */

  // Init date
  RTCDOW =  0x04;      /* Day of week = 0x04 = Thursday */
  RTCDAY =  0x0a;      /* Day = 0x0a = 10th*/
  RTCMON =  0x03;      /* Month = 0x03 = March */
  RTCYEAR = 0x2016;      /* Year = 0x2016*/
#endif

  RTCCTL01 &= ~(RTCHOLD);

  return RTCC_SUCCESS;
}
/*---------------------------------------------------------------------------*/
void
rtcc_init(void)
{
  /* Initialize interrupts handlers */
  rtcc_timer_callback = NULL;
  rtcc_alarm_callback = NULL;
  rtcc_clk_ready_callback = NULL;
}
/*---------------------------------------------------------------------------*/
