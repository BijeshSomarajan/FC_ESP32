#ifndef DELAYTIMER_H_
#define DELAYTIMER_H_
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include <rom/ets_sys.h>
#include "freertos/FreeRTOS.h"

void delayMs(uint32_t delay);
void delayUs(uint32_t delay);
uint8_t initDelayTimer();

#endif
