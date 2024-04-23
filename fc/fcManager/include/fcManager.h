#ifndef FCCOMPONENTS_FCMANAGER_H_
#define FCCOMPONENTS_FCMANAGER_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

void startFlightController(void);
void stopFlightController(uint8_t immediate);

#endif
