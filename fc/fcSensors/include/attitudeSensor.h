#ifndef FC_FCSENSORS_INCLUDE_ATTITUDESENSOR_H_
#define FC_FCSENSORS_INCLUDE_ATTITUDESENSOR_H_
#include <stdio.h>
#include <inttypes.h>
#include "lsm9ds1.h"
#include "fcLogger.h"

typedef struct _ATTITUDE_DATA ATTITUDE_DATA;
struct _ATTITUDE_DATA {
	float axG, ayG, azG, gxDS, gyDS, gzDS, mx, my, mz;
	float axGRaw, ayGRaw, azGRaw, gxDSRaw, gyDSRaw, gzDSRaw;
	float temp, tempRaw;
	float gyroDt;
	float accDt;
	float magDt;
	float tempDt;

	float accPitch;
	float accRoll;

	uint8_t cpu;
};

extern ATTITUDE_DATA attitudeData;

#define SENSOR_ACC_SAMPLE_PERIOD LSM9DS1_ACC_SAMPLE_PERIOD
#define SENSOR_GYRO_SAMPLE_PERIOD LSM9DS1_GYRO_SAMPLE_PERIOD
#define SENSOR_MAG_SAMPLE_PERIOD LSM9DS1_MAG_SAMPLE_PERIOD
#define SENSOR_TEMP_SAMPLE_PERIOD LSM9DS1_TEMP_SAMPLE_PERIOD

//Sensor Low Pass Frequencies
#define SENSOR_GYRO_LPF_FREQUENCY 400.0f //500
#define SENSOR_ACC_LPF_FREQUENCY  100.0f  //10
#define SENSOR_MAG_LPF_FREQUENCY  50.0f //100
#define SENSOR_TEMP_LPF_FREQUENCY 50.0f

//Acc Biquad NTF filter settings
#define SENSOR_GYRO_NTF_DEFAULT_CENTER_FREQ SENSOR_GYRO_LPF_FREQUENCY /// 2.0f
#define SENSOR_ACC_NTF_DEFAULT_CENTER_FREQ SENSOR_ACC_LPF_FREQUENCY // / 2.0f
#define SENSOR_ACC_GYRO_NOISE_GAIN_FACTOR 1.25f //No Rationale :(

#define SENSOR_ACC_BI_NTF_PEAK_GAIN 1.0f
#define SENSOR_ACC_BI_NTF_Q  0.33f //0.920f // Greater more > Sharp filtering

#define SENSOR_GYRO_BI_NTF_PEAK_GAIN 1.0f
#define SENSOR_GYRO_BI_NTF_Q  0.33f //0.920f // Greater more > Sharp filtering

//Acc Biquad Low Pass filter settings
#define SENSOR_ACC_BI_LPF_PEAK_GAIN  1.0f
#define SENSOR_ACC_BI_LPF_CENTER_FREQUENCY SENSOR_ACC_LPF_FREQUENCY
#define SENSOR_ACC_BI_LPF_Q 0.15f //0.25f

#define SENSOR_GYRO_BI_LPF_PEAK_GAIN 1.0f
#define SENSOR_GYRO_BI_LPF_CENTER_FREQUENCY SENSOR_GYRO_LPF_FREQUENCY
#define SENSOR_GYRO_BI_LPF_Q 0.15f // 0.25f

#define SENSOR_ACC_FLYABLE_VALUE_XY_LIMIT 3.0f // G
#define SENSOR_ACC_FLYABLE_VALUE_Z_LIMIT 4.0f // G
#define SENSOR_GYRO_FLYABLE_VALUE_LIMIT 200.0f //degrees/sec

/*------------------- Calibration related  -------------*/
#define SENSOR_AG_OFFSET_CALIB_SAMPLE_COUNT 2000.0f
#define SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY 2

#define SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY 2

#define LSM9DS1_TEMP_CAL_PROXIMITY_DEAD_BAND 1.0f
#define SESNSOR_TEMP_CAL_RANGE 45.0f //Degrees
#define SENSOR_TEMP_CAL_TEMP_DELTA 0.25f
#define SENSOR_AG_CALIB_LOWPASS_FREQ 0.1f
#define LSM9DS1_TEMP_CALIB_LOWPASS_FREQ 0.1f

#define SENSOR_MAG_CALIB_SAMPLE_COUNT 8000
#define SENSOR_MAG_CALIB_SAMPLE_DELAY 10
#define LSM9DS1_MAG_CALIB_USE_SIMPLE_ALGO 1

#define LSMDS1_TEMP_DELTA_SCALE_FACTOR  1.0f

uint8_t initAttitudeSensors(float motorKv, float batVolt, float motorNoiseFactor);
void readAccAndGyroSensor(float dt);
void readAccSensor(float dt);
void readGyroSensor(float dt);
void readTempSensor(float dt);
void readMagSensor(float dt);
void calculateAccAndGyroBias(void);
void calculateMagBias(void);
void calculateAccAndGyroTempCoeff(void);
float getMaxValidG(void);
void calculateMotorNoise(float throttlePercentage);
#endif
