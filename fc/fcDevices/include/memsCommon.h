#ifndef FC_FCDEVICES_INCLUDE_MEMSCOMMON_H_
#define FC_FCDEVICES_INCLUDE_MEMSCOMMON_H_
#include <stdio.h>
#include <inttypes.h>

typedef struct _MEMSDATA MEMSDATA;
struct _MEMSDATA {
	//Global buffer for read write operations
	uint8_t buffer[32];
	//Accelerometer , Gyroscope and Magnetometer Full Scale
	int16_t gyroFullScale, accFullScale, magFullScale;
	float maxValidG;
	float maxValidRotationRate;
	//Temperature
	int16_t rawTemp;
	//Temperature in Celcius
	float tempC;
	//Temperature offsets and coefficients
	float offsetTemp;
	double gyroXTempCoeff[4];
	double gyroYTempCoeff[4];
	double gyroZTempCoeff[4];

	double accXTempCoeff[4];
	double accYTempCoeff[4];
	double accZTempCoeff[4];

	float accXTempOffset;
	float accYTempOffset;
	float accZTempOffset;

	float gyroXTempOffset;
	float gyroYTempOffset;
	float gyroZTempOffset;

	//Accelerometer , Gyroscope and Magnetometer sensitivity
	float gyroSensitivity, accSensitivity, magSensitivity, tempSensitivity;
	//Accelerometer ,Gyroscope and Magnetometer Raw measurements
	int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz, rawMx, rawMy, rawMz;
	//Accelerometer , Gyroscope Raw offsets
	int16_t offsetAx, offsetAy, offsetAz, offsetGx, offsetGy, offsetGz;
	//Magnetometer factory offset , env bias and scale
	float offsetMx, offsetMy, offsetMz, biasMx, biasMy, biasMz, scaleMx, scaleMy, scaleMz;
	//Accelerometer , Gyroscope and Magnetometer Scaled measurements
	float axG, ayG, azG, gxDS, gyDS, gzDS, mx, my, mz;
};

#define MEMS_SENSOR_LSM9DS1 1
#define MEMS_SENSOR_MPU9250 2
#define MEMS_SENSOR_SELECTED MEMS_SENSOR_LSM9DS1

#if MEMS_SENSOR_SELECTED == MEMS_SENSOR_LSM9DS1
#define MEMS_ACC_SAMPLE_FREQUENCY 1000.0f
#define MEMS_GYRO_SAMPLE_FREQUENCY 4000.0f
#define MEMS_MAG_SAMPLE_FREQUENCY 500.0f
#define MEMS_TEMP_SAMPLE_FREQUENCY 20.0f

#define MEMS_APPLY_ACC_TEMP_OFFSET_CORRECTION 0
#define MEMS_APPLY_GYRO_TEMP_OFFSET_CORRECTION 1
#endif

#if MEMS_SENSOR_SELECTED == MEMS_SENSOR_MPU9250
#define MEMS_ACC_SAMPLE_FREQUENCY 800.0f
#define MEMS_GYRO_SAMPLE_FREQUENCY 4000.0f
#define MEMS_MAG_SAMPLE_FREQUENCY 400.0f
#define MEMS_TEMP_SAMPLE_FREQUENCY 20.0f

#define MEMS_APPLY_ACC_TEMP_OFFSET_CORRECTION 1
#define MEMS_APPLY_GYRO_TEMP_OFFSET_CORRECTION 1
#endif

uint8_t memsInit(void);
void memsReadAccAndGyro(void);
void memsReadAcc(void);
void memsReadGyro(void);
void memsReadTemp(void);
void memsReadMag(void);

void memsApplyAccOffsetCorrection(void);
void memsApplyGyroOffsetCorrection(void);
void memsApplyMagOffsetCorrection(void);

void memsApplyGyroTempOffsetCorrection(void);
void memsApplyAccTempOffsetCorrection(void);

void memsApplyAccDataScaling(void);
void memsApplyGyroDataScaling(void);
void memsApplyMagDataScaling(void);
void memsApplyTempDataScaling(void);

float getMemsMaxValidG(void);

#endif /* FC_FCDEVICES_INCLUDE_MEMSCOMMON_H_ */
