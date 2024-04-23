#ifndef DELTATIMER_H_
#define DELTATIMER_H_

#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_timer.h"

uint8_t initDeltaTimer(void);
float getDeltaTime(void);
void resetDeltaTime(void);
int64_t getTimeUSec(void);
float getUSecTimeInSec(int64_t usecTime);

#endif
