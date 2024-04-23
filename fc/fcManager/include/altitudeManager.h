#ifndef ALTITUDEMANAGER_H_
#define ALTITUDEMANAGER_H_
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "rcSensor.h"

#define ALTITUDE_MGR_USE_TIMER 0

#define SENSOR_ALT_SAMPLE_PERIOD_US SENSOR_ALT_SAMPLE_PERIOD * 1000000.0f

//Max permissible throttle
#define ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA 900
#define ALT_MGR_MAX_PERMISSIBLE_THROTTLE  RC_CHANNEL_MIN_VALUE + ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA
#define ALT_MGR_THROTTLE_ALT_AGGREGATION_GAIN  0.05f //0.0215f
#define ALT_MGR_THROTTLE_AGGREGATION_FREQUENCY 50.0f
#define ALT_MGR_THROTTLE_AGGREGATION_PERIOD   1.0f/ALT_MGR_THROTTLE_AGGREGATION_FREQUENCY

#define ALT_MGR_THROTTLE_CONTROL_LPF_FREQ  0.65f//0.65f

/* Vertical Velocity configurations */
#define ALT_MGR_VERTICAL_VELOCITY_DEADBAND     0.2f //5.0f
#define ALT_MGR_VERTICAL_VELOCITY_INPUT_GAIN   9.8f
#define ALT_MGR_VERTICAL_VELOCITY_OUTPUT_GAIN  9.8f
#define ALT_MGR_VERTICAL_VELOCITY_MAX        100.0f
#define ALT_MGR_VERTICAL_VELOCITY_LPF_FREQ   0.90f

#define ALT_MGR_ALT_HOLD_ACTIVATION_PERIOD   0.3f
#define ALT_MGR_ALT_MAX_DISTANCE_DELTA  15.0f //100 cms

uint8_t initAltitudeManager(void);
uint8_t startAltitudeManager(void);
uint8_t stopAltitudeManager(void);
uint8_t resetAltitudeManager(void);

#endif
