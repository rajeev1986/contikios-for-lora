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
 *         rtcc.h
 * \author
 *         David Rodenas-Herraiz <dr424@cam.ac.uk>
 *
 * \brief
 *         Driver for real-time clock RTC_B module in MSP430FR5969
 *
 * \note   This file is adapted from the RE-Mote on-board ultra-low power RTCC
 *         driver
 */

/* -------------------------------------------------------------------------- */
#ifndef RTCC_H_
#define RTCC_H_
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* Time/date registers (no offset) */
#define SEC_ADDR               0x00
#define MIN_ADDR               0x01
#define HOUR_ADDR              0x02
#define DAY_ADDR               0x03
#define MONTHS_ADDR            0x04
#define YEAR_ADDR              0x05
#define WEEKDAYLS_ADDR         0x06

/* Alarm registers */
#define MINUTES_ALARM_ADDR     0x00
#define HOURS_ALARM_ADDR       0x01
#define DAY_ALARMS_ADDR        0x02
#define WEEKDAYS_ALARM_ADDR    0x03

/* -------------------------------------------------------------------------- */
#define RTCC_TD_MAP_SIZE           (WEEKDAYLS_ADDR + 1)
#define RTCC_ALARM_MAP_SIZE        (WEEKDAYS_ALARM_ADDR + 1)
/* -------------------------------------------------------------------------- */
enum {
  RTCC_24H_MODE = 0,
  RTCC_12H_MODE_AM,
  RTCC_12H_MODE_PM,
};
/* -------------------------------------------------------------------------- */
enum {
  RTCC_TIMER_MIN = 0,
  RTCC_TIMER_HOUR,
  RTCC_TIMER_MIDNIGHT,
  RTCC_TIMER_NOON,
};
/* -------------------------------------------------------------------------- */
/** \name RTCC error values
 * @{
 */
#define RTCC_ERROR                 (-1)
#define RTCC_SUCCESS               0x00
/* -------------------------------------------------------------------------- */
/** \name Readable Date and time memory map implementation
 *
 * This simplified structure allows the user to set date/alarms with a
 * reduced structure, without the bit-defined restrictions of the memory map,
 * using decimal values
 *
 * @{
 */
typedef struct rtcB_struct_simple_td_reg {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint8_t day;
  uint8_t months;
  uint8_t years;
  uint8_t weekdays;
  uint8_t mode;
}  __attribute__ ((packed)) simple_td_map;
/* -------------------------------------------------------------------------- */
/**
 * \name Callback function to handle the RTCC alarm interrupt and macro
 * @{
 */
#define RTCC_REGISTER_TIMER(ptr) rtcc_timer_callback = ptr;
extern void (*rtcc_timer_callback)(uint8_t value);

#define RTCC_REGISTER_ALARM(ptr) rtcc_alarm_callback = ptr;
extern void (*rtcc_alarm_callback)(uint8_t value);

#define RTCC_REGISTER_CLK_READY(ptr) rtcc_clk_ready_callback = ptr;
extern void (*rtcc_clk_ready_callback)(uint8_t value);
/* -------------------------------------------------------------------------- */

/**
 * \brief Set the time and date
 * \param *data Time and date value (decimal format)
 * \return
 * \           RTCC_SUCCESS date/time set
 * \           RTCC_ERROR failed to set time/date (enable DEBUG for more info)
 */
int8_t rtcc_set_time_date(simple_td_map *data);

/**
 * \brief Get the current time and date
 * \param *data buffer to store the results
 * \return
 * \           RTCC_SUCCESS date/time set
 * \           RTCC_ERROR failed to set time/date (enable DEBUG for more info)
 */
int8_t rtcc_get_time_date(simple_td_map *data);

/**
 * \brief Configure the RTCC to match an alarm counter
 * \param data date and time values (in decimal) to match against
 * \return
 * \           RTCC_SUCCESS date/time set
 * \           RTCC_ERROR failed to set time/date (enable DEBUG for more info)
 */
int8_t rtcc_set_alarm_time_date(simple_td_map *data);

/**
 * \brief Configure the RTCC to match a timer counter
 * \param real-time clock time interrupt event interval
 * \return
 * \           RTCC_SUCCESS date/time set
 * \           RTCC_ERROR failed to set time/date (enable DEBUG for more info)
 */
int8_t rtcc_init_timer(uint8_t RTCTEV_bits);

/**
 * \brief Initialize the RTCC
 */
void rtcc_init(void);
/** @} */
/* -------------------------------------------------------------------------- */
#endif /* ifndef RTCC_H_ */
/* -------------------------------------------------------------------------- */
/**
 * @}
 * @}
 */