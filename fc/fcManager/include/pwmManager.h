#ifndef MODULES_FC_PWM_H_
#define MODULES_FC_PWM_H_

#include <stdio.h>
#include <stdint.h>
#include "attitudeSensor.h"
#include "attitudeManager.h"

#define PWM_MGR_USE_TIMER 1
#define PWM_UPDATE_PERIOD  ATTITUDE_CONTROL_UPDATE_PERIOD
#define PWM_UPDATE_PERIOD_US PWM_UPDATE_PERIOD * 1000000.0f

uint8_t initPWMManager(void);
uint8_t startPWMManager(void);
uint8_t stopPWMManager(void);

#endif
