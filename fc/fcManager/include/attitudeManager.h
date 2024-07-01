#ifndef ATTITUDEMANAGER_H_
#define ATTITUDEMANAGER_H_
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#define SENSOR_READ_ACC_AND_GYRO_TOGETHER 0

#define SENSOR_ACC_SAMPLE_PERIOD_US SENSOR_ACC_SAMPLE_PERIOD * 1000000.0f
#define SENSOR_GYRO_SAMPLE_PERIOD_US SENSOR_GYRO_SAMPLE_PERIOD * 1000000.0f
#define SENSOR_MAG_SAMPLE_PERIOD_US SENSOR_MAG_SAMPLE_PERIOD * 1000000.0f
#define SENSOR_TEMP_SAMPLE_PERIOD_US SENSOR_TEMP_SAMPLE_PERIOD * 1000000.0f

#define ATTITUDE_IMU_UPDATE_PERIOD SENSOR_GYRO_SAMPLE_PERIOD * 4.0f
#define ATTITUDE_IMU_UPDATE_PERIOD_US (ATTITUDE_IMU_UPDATE_PERIOD * 1000000.0f)

#define ATTITUDE_CONTROL_UPDATE_PERIOD SENSOR_GYRO_SAMPLE_PERIOD * 2.0f
#define ATTITUDE_CONTROL_UPDATE_PERIOD_US  (ATTITUDE_CONTROL_UPDATE_PERIOD * 1000000.0f)

#define ATTITUDE_STABILIZATION_PERIOD 10.0f //10 Seconds
#define ATTITUDE_CONTROL_MIN_TH_PERCENT 0.1f //10%

#define ATT_MAX_RC_ANGLE_FACTOR  8.0f
#define ATT_MAX_PITCH_ROLL  10.0f * ATT_MAX_RC_ANGLE_FACTOR

#define MOTOR_KV 2400
#define BATTERY_VOLT 11.1f
#define MOTOR_NUMBER 4.0f

uint8_t initAttitudeManager(void);
uint8_t startAttitudeManager(void);
uint8_t stopAttitudeManager(void);
uint8_t resetAttitudeManager(void);

void calibarateAGForBias(void);
void calibarateMagForBias(void);
void calibarateAccAndGyroForTemp(void);

#endif
