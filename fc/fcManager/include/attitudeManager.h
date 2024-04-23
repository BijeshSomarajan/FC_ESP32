#ifndef ATTITUDEMANAGER_H_
#define ATTITUDEMANAGER_H_
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#define SENSOR_READ_ACC_AND_GYRO_TOGETHER 1

#define SENSOR_ACC_SAMPLE_PERIOD_US SENSOR_ACC_SAMPLE_PERIOD * 1000000.0f
#define SENSOR_GYRO_SAMPLE_PERIOD_US SENSOR_GYRO_SAMPLE_PERIOD * 1000000.0f
#define SENSOR_MAG_SAMPLE_PERIOD_US SENSOR_MAG_SAMPLE_PERIOD * 1000000.0f
#define SENSOR_TEMP_SAMPLE_PERIOD_US SENSOR_TEMP_SAMPLE_PERIOD * 1000000.0f

#define ATTITUDE_IMU_UPDATE_PERIOD SENSOR_GYRO_SAMPLE_PERIOD
#define ATTITUDE_IMU_UPDATE_PERIOD_US (ATTITUDE_IMU_UPDATE_PERIOD * 1000000.0f)/5.0f

#define ATTITUDE_CONTROL_UPDATE_PERIOD 1.0f/1000.0f
#define ATTITUDE_CONTROL_UPDATE_PERIOD_US ATTITUDE_CONTROL_UPDATE_PERIOD * 1000000.0f

uint8_t initAttitudeManager(void);
uint8_t startAttitudeManager(void);
uint8_t stopAttitudeManager(void);
uint8_t resetAttitudeManager(void);

void calibarateAGForBias(void);
void calibarateMagForBias(void);
void calibarateAccAndGyroForTemp(void);

#endif
