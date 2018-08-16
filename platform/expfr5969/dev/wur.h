#ifndef WUR_H
#define WUR_H

#include "lib/sensors.h"

extern const struct sensors_sensor wur_sensor;

#define WUR_SENSOR "wur_rx_sensor"

void wur_init();
void wur_set_tx();
void wur_clear_tx();

void wur_rx_done();
void wur_clear_rx();

#endif
