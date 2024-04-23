#ifndef _INDICATOR_SENSOR_H_
#define _INDICATOR_SENSOR_H

#include <stdio.h>
#include <inttypes.h>
#include "indicator.h"

uint8_t initIndicatorSensors(void);
void statusIndicatorSensorOn(void);
void statusIndicatorSensorOff(void);
void statusIndicatorSensorToggle(void);

#endif
