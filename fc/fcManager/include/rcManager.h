#ifndef MODULES_RADIO_FC_RC_H_
#define MODULES_RADIO_FC_RC_H_

#include <stdio.h>
#include <stdint.h>

#define RC_MGR_USE_TIMER 1
#define RC_DATA_READ_PERIOD  1.0f/2000.0f
#define RC_DATA_READ_PERIOD_US RC_DATA_READ_PERIOD * 1000000.0f

#define RC_DATA_PROCESS_PERIOD  1.0f/100.0f
#define RC_DATA_PROCESS_PERIOD_US RC_DATA_PROCESS_PERIOD * 1000000.0f

#define RC_FAIL_SAFE_ACTIVATION_PERIOD  1.0f // Fail safe activated after a second
#define THROTTLE_CENTER_DEADBAND  10
#define YAW_CENTER_DEADBAND  10
#define PITCH_CENTER_DEADBAND  10
#define ROLL_CENTER_DEADBAND  10
#define RC_CALIBRATION_COUNT  100

uint8_t initRCManager(void);
uint8_t startRCManager(void);
uint8_t stopRCManager(void);

int16_t getThrottleChannelValue();
int16_t getPitchChannelValue();
int16_t getRollChannelValue();
int16_t getYawChannelValue();

void resetRCManager(void);
uint8_t canArmModel(void);
void calibrateRC(void);

#endif
