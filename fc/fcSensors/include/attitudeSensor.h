#ifndef _ATTITUDESENSOR_H_
#define _ATTITUDESENSOR_H_
#include <stdio.h>
#include <inttypes.h>
#include "memsCommon.h"
#include "fcLogger.h"

#define MEMS_SENSOR_LSM9DS1 1
//#define MEMS_SENSOR_MPU9250 1

typedef struct _ATTITUDE_DATA ATTITUDE_DATA;
struct _ATTITUDE_DATA {
	volatile float axG, ayG, azG, gxDS, gyDS, gzDS, mx, my, mz;
	volatile float axGRaw, ayGRaw, azGRaw, gxDSRaw, gyDSRaw, gzDSRaw;
	volatile float temp, tempRaw;
	float gyroDt;
	float accDt;
	float magDt;
	float tempDt;

	float accPitch;
	float accRoll;

	uint8_t cpu;
};

extern ATTITUDE_DATA attitudeData;

#define SENSOR_ACC_SAMPLE_FREQUENCY MEMS_ACC_SAMPLE_FREQUENCY
#define SENSOR_GYRO_SAMPLE_FREQUENCY MEMS_GYRO_SAMPLE_FREQUENCY
#define SENSOR_MAG_SAMPLE_FREQUENCY MEMS_MAG_SAMPLE_FREQUENCY
#define SENSOR_TEMP_SAMPLE_FREQUENCY MEMS_TEMP_SAMPLE_FREQUENCY

#define SENSOR_ACC_SAMPLE_PERIOD 1.0f / SENSOR_ACC_SAMPLE_FREQUENCY
#define SENSOR_GYRO_SAMPLE_PERIOD 1.0f / SENSOR_GYRO_SAMPLE_FREQUENCY
#define SENSOR_MAG_SAMPLE_PERIOD  1.0f / SENSOR_MAG_SAMPLE_FREQUENCY
#define SENSOR_TEMP_SAMPLE_PERIOD  1.0f / SENSOR_TEMP_SAMPLE_FREQUENCY

#define SENSOR_ACC_FLYABLE_VALUE_XY_LIMIT 3.0f // G
#define SENSOR_ACC_FLYABLE_VALUE_Z_LIMIT 4.0f // G
#define SENSOR_GYRO_FLYABLE_VALUE_LIMIT 200.0f //degrees/sec

/*------------------- Calibration related  -------------*/
#define SENSOR_AG_OFFSET_CALIB_SAMPLE_COUNT 2000.0f
#define SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY 2

#define SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY 2

#define SESNSOR_TEMP_CAL_PROXIMITY_DEAD_BAND 1.0f
#define SESNSOR_TEMP_CAL_RANGE 45.0f //Degrees
#define SENSOR_TEMP_CAL_TEMP_DELTA 0.25f
#define SENSOR_AG_CALIB_LOWPASS_FREQ 0.1f
#define SENSOR_AG_TEMP_CALIB_LOWPASS_FREQ 0.1f

#define SENSOR_MAG_CALIB_SAMPLE_COUNT 8000
#define SENSOR_MAG_CALIB_SAMPLE_DELAY 10
#define SENSOR_MAG_CALIB_USE_SIMPLE_ALGO 1

uint8_t initAttitudeSensors(float motorKv, float batVolt, float nMotors,float nPropPairs);
void readAccAndGyroSensor(float dt);
void readAccSensor(float dt);
void readGyroSensor(float dt);
void readTempSensor(float dt);
void readMagSensor(float dt);
void calculateAccAndGyroBias(void);
void calculateMagBias(void);
void calculateAccAndGyroTempCoeff(void);
float getMaxValidG(void);

#endif
