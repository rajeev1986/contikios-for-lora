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
 * \author
 *         Rajeev Piyare <rajeev.piyare@hotmail.com>
 */
#include "contiki.h"
#include "lib/sensors.h"
#include "dev/hwconf.h"
#include "dev/button-sensor.h"
#include "isr_compat.h"

const struct sensors_sensor button_sensor;

static struct timer debouncetimer;
static int status(int type);

/* Button port for LoRa + WuR platform V3.0 ->P3.1 */
HWCONF_PINx(BUTTON, 3, 1, 0);
HWCONF_IRQx(BUTTON, 3, 1);

/*---------------------------------------------------------------------------*/
ISR(PORT3, irq_p3)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  if(BUTTON_CHECK_IRQ()) {
   if(timer_expired(&debouncetimer)) {
      timer_set(&debouncetimer, CLOCK_SECOND / 4);
      sensors_changed(&button_sensor);
      LPM4_EXIT;
    }
  }
  BUTTON_CLEAR_IRQ();
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  return BUTTON_READ() || !timer_expired(&debouncetimer);
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int c)
{
  switch (type) {
  case SENSORS_ACTIVE:
    if (c) {
      if(!status(SENSORS_ACTIVE)) {

      timer_set(&debouncetimer, 0);

      BUTTON_SET();
      BUTTON_MAKE_INPUT();
      BUTTON_ENABLE_PULLUP();
      BUTTON_IRQ_EDGE_SELECTD();
      BUTTON_ENABLE_IRQ();
      BUTTON_CLEAR_IRQ();
      }
    } else {
      BUTTON_DISABLE_IRQ();
    }
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch (type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return BUTTON_IRQ_ENABLED();
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor, BUTTON_SENSOR, value, configure, status);


