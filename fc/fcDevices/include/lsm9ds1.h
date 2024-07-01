#ifndef DEVICES_LSM9DS1_H_
#define DEVICES_LSM9DS1_H_

#include <stdio.h>
#include <inttypes.h>

typedef struct _LSM9DS1 LSM9DS1;
struct _LSM9DS1 {
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
//Global variable
extern LSM9DS1 lsm9ds1;

typedef enum {
	//ODR_XL2 ODR_XL1 ODR_XL0 FS1_XL FS0_XL BW_SCAL _ODR BW_XL1 BW_XL0
	LSM9DS1_ACCELRANGE_2G = 0xC4, //Acc ODR:952 , FS:2G , Filter : 408 Hz;
	LSM9DS1_ACCELRANGE_4G = 0xD4, //Acc ODR:952 , FS:4G , Filter : 408 Hz;
	LSM9DS1_ACCELRANGE_8G = 0xDC, //Acc ODR:952 , FS:8G , Filter : 408 Hz;
	LSM9DS1_ACCELRANGE_16G = 0xCC //Acc ODR:952 , FS:16G , Filter : 408 Hz;
} LSM9DS1AccelRange;

typedef enum {
	LSM9DS1_GYRORANGE_245DPS = 0xC3, //Gyro ODR : 952 , FS:245 , Filter : 100 Hz
	LSM9DS1_GYRORANGE_500DPS = 0xCB, //Gyro ODR : 952 , FS:500 , Filter : 100 Hz
	LSM9DS1_GYRORANGE_2000DPS = 0xDB //Gyro ODR : 952 , FS:2000 , Filter : 100 Hz
} LSM9DS1GyroRange;

typedef enum {
	LSM9DS1_MAGRANGE_4GAUSS = 0x0, //4 Gauss
	LSM9DS1_MAGRANGE_8GAUSS = 0x20, //8 Gauss
	LSM9DS1_MAGRANGE_16GAUSS = 0x60 //16 Gauss
} LSM9DS1MagRange;

#define LSM9DS1_ACC_SAMPLE_FREQUENCY 1000.0f
#define LSM9DS1_ACC_SAMPLE_PERIOD 1.0f / LSM9DS1_ACC_SAMPLE_FREQUENCY

#define LSM9DS1_GYRO_SAMPLE_FREQUENCY 4000.0f
#define LSM9DS1_GYRO_SAMPLE_PERIOD 1.0f / LSM9DS1_GYRO_SAMPLE_FREQUENCY

#define LSM9DS1_MAG_SAMPLE_FREQUENCY 500.0f
#define LSM9DS1_MAG_SAMPLE_PERIOD  1.0f / LSM9DS1_MAG_SAMPLE_FREQUENCY

#define LSM9DS1_TEMP_SAMPLE_FREQUENCY 20.0f
#define LSM9DS1_TEMP_SAMPLE_PERIOD  1.0f / LSM9DS1_TEMP_SAMPLE_FREQUENCY

#define LSM9DS1_APPLY_ACC_TEMP_OFFSET_CORRECTION 0
#define LSM9DS1_APPLY_GYRO_TEMP_OFFSET_CORRECTION 1

uint8_t initlsm9ds1(void);

void lsm9ds1ReadAccAndGyro(void);
void lsm9ds1ReadAcc(void);
void lsm9ds1ReadGyro(void);
void lsm9ds1ReadTemp(void);
void lsm9ds1ReadMag(void);

void lsm9ds1ApplyAccOffsetCorrection(void);
void lsm9ds1ApplyGyroOffsetCorrection(void);
void lsm9ds1ApplyMagOffsetCorrection(void);

void lsm9ds1ApplyGyroTempOffsetCorrection(void);
void lsm9ds1ApplyAccTempOffsetCorrection(void);

void lsm9ds1ApplyAccDataScaling(void);
void lsm9ds1ApplyGyroDataScaling(void);
void lsm9ds1ApplyMagDataScaling(void);

float getLsm9ds1MaxValidG(void);

#endif
