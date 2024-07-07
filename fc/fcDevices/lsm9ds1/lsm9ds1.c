#include "memsCommon.h"
#include "lsm9ds1Registers.h"
#include "fcSPI.h"
#include "delayTimer.h"
#include "fcLogger.h"

#if MEMS_SENSOR_SELECTED == MEMS_SENSOR_LSM9DS1

#define LSM9DS1_AG_DEVICE SPI3_DEVICE_1
#define LSM9DS1_MAG_DEVICE SPI3_DEVICE_2

extern MEMSDATA memsData;

uint8_t memsAGTCheckConnection(void);
uint8_t memsMCheckConnection(void);
uint8_t memsAGInit(int16_t pGyroFullScale, int16_t pAccFullScale);
uint8_t memsMInit(int16_t pMagFullScale);

uint8_t memsInit() {
	uint8_t status = initSPI3();
	if (status) {
		logString("[lsm9ds1,IO:SPI-3] > Success\n");
	} else {
		logString("[lsm9ds1,IO:SPI-3] > Failed\n");
		return 0;
	}
	status = memsAGTCheckConnection();
	if (status) {
		logString("[lsm9ds1,AGT:CON] > Success\n");
	} else {
		logString("[lsm9ds1,AGT:CON] > Failed\n");
		return 0;
	}
	status = memsMCheckConnection();
	if (status) {
		logString("[lsm9ds1,MAG:CON] > Success\n");
	} else {
		logString("[lsm9ds1,MAG:CON] > Failed\n");
		return 0;
	}
	status = memsAGInit(LSM9DS1_GYRORANGE_245DPS, LSM9DS1_ACCELRANGE_8G);
	if (status) {
		logString("[lsm9ds1,AGT:Init] > Success\n");
	} else {
		logString("[lsm9ds1,AGT:Init] > Failed\n");
		return 0;
	}
	status = memsMInit(LSM9DS1_MAGRANGE_8GAUSS);
	if (status) {
		logString("[lsm9ds1,MAG:Init] > Success\n");
	} else {
		logString("[lsm9ds1,MAG:Init] > Failed\n");
		return 0;
	}
	return status;
}

uint8_t memsAGTCheckConnection() {
	uint8_t status = spi3Read(LSM9DS1_WHO_AM_I_XG, memsData.buffer, 1, LSM9DS1_AG_DEVICE);
	if (status) {
		status = (memsData.buffer[0] == LSM9DS1_WHO_AM_I_AG_RSP);
	}
	return status;
}

uint8_t memsMCheckConnection() {
	uint8_t status = spi3Read(LSM9DS1_WHO_AM_I_M, memsData.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (status) {
		status = (memsData.buffer[0] == LSM9DS1_WHO_AM_I_M_RSP);
	}
	return status;
}

float getMemsMaxValidG() {
	return memsData.maxValidG * 0.85f;
}

void memsDeriveGyroSensitivity() {
	switch (memsData.gyroFullScale) {
	case LSM9DS1_GYRORANGE_245DPS:
		memsData.gyroSensitivity = 245.0f / 32768.0f;
		memsData.maxValidRotationRate = 245.0f;
		break;
	case LSM9DS1_GYRORANGE_500DPS:
		memsData.gyroSensitivity = 500.0f / 32768.0f;
		memsData.maxValidRotationRate = 500.0f;
		break;
	case LSM9DS1_GYRORANGE_2000DPS:
		memsData.gyroSensitivity = 2000.0f / 32768.0f;
		memsData.maxValidRotationRate = 2000.0f;
		break;
	}
}

void memsDeriveAccSensitivity() {
	switch (memsData.accFullScale) {
	case LSM9DS1_ACCELRANGE_2G:
		memsData.accSensitivity = 2.0f / 32768.0f;
		memsData.maxValidG = 2.0f;
		break;
	case LSM9DS1_ACCELRANGE_4G:
		memsData.accSensitivity = 4.0f / 32768.0f;
		memsData.maxValidG = 4.0f;
		break;
	case LSM9DS1_ACCELRANGE_8G:
		memsData.accSensitivity = 8.0f / 32768.0f;
		memsData.maxValidG = 8.0f;
		break;
	case LSM9DS1_ACCELRANGE_16G:
		memsData.accSensitivity = 16.0f / 32768.0f;
		memsData.maxValidG = 16.0f;
		break;
	}
}

void memsDeriveMagSensitivity() {
	switch (memsData.magFullScale) {
	case LSM9DS1_MAGRANGE_4GAUSS:
		memsData.magSensitivity = 0.00014f;
		break;
	case LSM9DS1_MAGRANGE_8GAUSS:
		memsData.magSensitivity = 0.00029f;
		break;
	case LSM9DS1_MAGRANGE_16GAUSS:
		memsData.magSensitivity = 0.00058f;
		break;
	}
}

uint8_t memsAGInit(int16_t pGyroFullScale, int16_t pAccFullScale) {
	uint8_t status = 0;

	memsData.gyroFullScale = pGyroFullScale;
	memsData.accFullScale = pAccFullScale;

	//Reset and allow burst reads
	memsData.buffer[0] = 0x1; //00000001
	status = spi3Write(LSM9DS1_CTRL_REG8, memsData.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Gyro ODR : 952 , FS:245 , Filter : 100 Hz
	//110 00 0 11
	memsData.buffer[0] = memsData.gyroFullScale;
	status = spi3Write(LSM9DS1_CTRL_REG1_G, memsData.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Disable Fifo
	memsData.buffer[0] = 0x0;
	status = spi3Write(LSM9DS1_FIFO_CTRL, memsData.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//0000[int 0:1][out sel 0:1]
	//0000 00 00
	memsData.buffer[0] = 0x0;
	status = spi3Write(LSM9DS1_CTRL_REG2_G, memsData.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//LP_mode HP_EN 0(1) 0(1) HPCF3_G HPCF2_G HPCF1_G HPCF0_G
	//0 0 00 1001
	memsData.buffer[0] = 0x9;
	status = spi3Write(LSM9DS1_CTRL_REG3_G, memsData.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Enable XYZ accelerometer no decimation
	memsData.buffer[0] = 0x38;
	status = spi3Write(LSM9DS1_CTRL_REG5_XL, memsData.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	memsData.buffer[0] = memsData.accFullScale;
	status = spi3Write(LSM9DS1_CTRL_REG6_XL, memsData.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//HR DCF1 DCF0 0(1) 0(1) FDS 0(1) HPIS1
	//1 10 0 00 0 0
	memsData.buffer[0] = 0xC0;
	status = spi3Write(LSM9DS1_CTRL_REG7_XL, memsData.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	memsData.tempSensitivity = 1.0 / 16.0f;
	memsDeriveGyroSensitivity();
	memsDeriveAccSensitivity();

	return status;
}

uint8_t memsMInit(int16_t pMagFullScale) {
	uint8_t status = 0;
	memsData.magFullScale = pMagFullScale;
	//Soft Reset
	memsData.buffer[0] = 0x4;
	status = spi3Write(LSM9DS1_CTRL_REG2_M, memsData.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Temp compensation enabled , high perf , ODR : 80Hz , Fast mode
	memsData.buffer[0] = 0xDE;
	status = spi3Write(LSM9DS1_CTRL_REG1_M, memsData.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	memsData.buffer[0] = pMagFullScale;
	status = spi3Write(LSM9DS1_CTRL_REG2_M, memsData.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//I2C disable, no low power , continuous mode
	memsData.buffer[0] = 0x80;
	status = spi3Write(LSM9DS1_CTRL_REG3_M, memsData.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Z axis High performance mode
	memsData.buffer[0] = 0x8;
	status = spi3Write(LSM9DS1_CTRL_REG4_M, memsData.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Fast read disabled , continuous mode
	memsData.buffer[0] = 0x0;
	status = spi3Write(LSM9DS1_CTRL_REG5_M, memsData.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	memsDeriveMagSensitivity();
	return status;
}

void memsApplyAccOffsetCorrection() {
	memsData.rawAx -= memsData.offsetAx;
	memsData.rawAy -= memsData.offsetAy;
	memsData.rawAz -= memsData.offsetAz;
}

void memsApplyGyroOffsetCorrection() {
	memsData.rawGx -= memsData.offsetGx;
	memsData.rawGy -= memsData.offsetGy;
	memsData.rawGz -= memsData.offsetGz;
}

void memsApplyAccDataScaling() {
	memsData.axG = (float) memsData.rawAx * memsData.accSensitivity;
	memsData.ayG = (float) memsData.rawAy * memsData.accSensitivity;
	memsData.azG = (float) memsData.rawAz * memsData.accSensitivity;
}

void memsApplyGyroDataScaling() {
	memsData.gxDS = (float) memsData.rawGx * memsData.gyroSensitivity;
	memsData.gyDS = (float) memsData.rawGy * memsData.gyroSensitivity;
	memsData.gzDS = (float) memsData.rawGz * memsData.gyroSensitivity;
}

void memsApplyMagDataScaling() {
	memsData.mx = ((float) memsData.rawMx * memsData.magSensitivity);
	memsData.my = ((float) memsData.rawMy * memsData.magSensitivity);
	memsData.mz = ((float) memsData.rawMz * memsData.magSensitivity);
}

void memsApplyAccTempOffsetCorrection() {
#if LSM9DS1_APPLY_ACC_TEMP_OFFSET_CORRECTION ==1
	memsData.axG -= memsData.accXTempOffset;
	memsData.ayG -= memsData.accYTempOffset;
	memsData.azG -= memsData.accZTempOffset;
#endif
}

void memsApplyGyroTempOffsetCorrection() {
#if LSM9DS1_APPLY_GYRO_TEMP_OFFSET_CORRECTION ==1
	memsData.gxDS -= memsData.gyroXTempOffset;
	memsData.gyDS -= memsData.gyroYTempOffset;
	memsData.gzDS -= memsData.gyroZTempOffset;
#endif
}

void memsApplyMagOffsetCorrection() {
	/*
	 memsData.mx *= memsData.offsetMx;
	 memsData.my *= memsData.offsetMy;
	 memsData.mz *= memsData.offsetMz;
	 */
	//Apply the bias if already set
	memsData.mx -= memsData.biasMx;
	memsData.my -= memsData.biasMy;
	memsData.mz -= memsData.biasMz;
	//Apply the scales for each axis
	memsData.mx *= memsData.scaleMx;
	memsData.my *= memsData.scaleMy;
	memsData.mz *= memsData.scaleMz;
}

void memsReadGyro() {
	if (spi3Read(LSM9DS1_OUT_X_L_G, memsData.buffer, 6, LSM9DS1_AG_DEVICE)) {
		memsData.rawGx = (((int16_t) memsData.buffer[1]) << 8) | (memsData.buffer[0] & 0xFF);
		memsData.rawGy = (((int16_t) memsData.buffer[3]) << 8) | (memsData.buffer[2] & 0xFF);
		memsData.rawGz = (((int16_t) memsData.buffer[5]) << 8) | (memsData.buffer[4] & 0xFF);
	}
}

void memsReadAcc() {
	if (spi3Read(LSM9DS1_OUT_X_L_XL, memsData.buffer, 6, LSM9DS1_AG_DEVICE)) {
		memsData.rawAx = (((int16_t) memsData.buffer[1]) << 8) | (memsData.buffer[0] & 0xFF);
		memsData.rawAy = (((int16_t) memsData.buffer[3]) << 8) | (memsData.buffer[2] & 0xFF);
		memsData.rawAz = (((int16_t) memsData.buffer[5]) << 8) | (memsData.buffer[4] & 0xFF);
	}
}

void memsReadAccAndGyro() {
	if (spi3Read(LSM9DS1_OUT_X_L_G, memsData.buffer, 12, LSM9DS1_AG_DEVICE)) {
		memsData.rawGx = (((int16_t) memsData.buffer[1]) << 8) | (memsData.buffer[0] & 0xFF);
		memsData.rawGy = (((int16_t) memsData.buffer[3]) << 8) | (memsData.buffer[2] & 0xFF);
		memsData.rawGz = (((int16_t) memsData.buffer[5]) << 8) | (memsData.buffer[4] & 0xFF);
		memsData.rawAx = (((int16_t) memsData.buffer[7]) << 8) | (memsData.buffer[6] & 0xFF);
		memsData.rawAy = (((int16_t) memsData.buffer[9]) << 8) | (memsData.buffer[8] & 0xFF);
		memsData.rawAz = (((int16_t) memsData.buffer[11]) << 8) | (memsData.buffer[10] & 0xFF);
	}
}

void memsReadTemp() {
	if (spi3Read(LSM9DS1_OUT_TEMP_L, memsData.buffer, 2, LSM9DS1_AG_DEVICE)) {
		memsData.rawTemp = (((int16_t) memsData.buffer[1]) << 8) | (memsData.buffer[0] & 0xFF);
	}
}

void memsApplyTempDataScaling() {
	memsData.tempC = (float) memsData.rawTemp * memsData.tempSensitivity + 25.0f;
}

void memsReadMag() {
	if (spi3Read((LSM9DS1_OUT_READ_MULTIPLE | LSM9DS1_OUT_X_L_M), memsData.buffer, 6, LSM9DS1_MAG_DEVICE)) {
		memsData.rawMx = (((int16_t) memsData.buffer[1]) << 8) | (memsData.buffer[0] & 0xFF);
		memsData.rawMy = (((int16_t) memsData.buffer[3]) << 8) | (memsData.buffer[2] & 0xFF);
		memsData.rawMz = (((int16_t) memsData.buffer[5]) << 8) | (memsData.buffer[4] & 0xFF);
	}
}

#endif
