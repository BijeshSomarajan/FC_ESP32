#include "lsm9ds1.h"
#include "lsm9ds1Config.h"
#include "lsm9ds1Registers.h"
#include "fcSPI.h"
#include "delayTimer.h"
#include "fcLogger.h"

uint8_t lsm9ds1AGTCheckConnection(void);
uint8_t lsm9ds1MCheckConnection(void);
uint8_t lsm9ds1AGInit(int16_t pGyroFullScale, int16_t pAccFullScale);
uint8_t lsm9ds1MInit(int16_t pMagFullScale);

LSM9DS1 lsm9ds1;

uint8_t initlsm9ds1() {
	uint8_t status = initSPI3();
	if (status) {
		logString("[lsm9ds1,IO:SPI-3] > Success\n");
	} else {
		logString("[lsm9ds1,IO:SPI-3] > Failed\n");
		return 0;
	}

	status = lsm9ds1AGTCheckConnection();
	if (status) {
		logString("[lsm9ds1,AGT:CON] > Success\n");
	} else {
		logString("[lsm9ds1,AGT:CON] > Failed\n");
		return 0;
	}

	status = lsm9ds1MCheckConnection();
	if (status) {
		logString("[lsm9ds1,MAG:CON] > Success\n");
	} else {
		logString("[lsm9ds1,MAG:CON] > Failed\n");
		return 0;
	}

	status = lsm9ds1AGInit(LSM9DS1_GYRORANGE_245DPS, LSM9DS1_ACCELRANGE_8G);
	if (status) {
		logString("[lsm9ds1,AGT:Init] > Success\n");
	} else {
		logString("[lsm9ds1,AGT:Init] > Failed\n");
		return 0;
	}

	status = lsm9ds1MInit(LSM9DS1_MAGRANGE_8GAUSS);
	if (status) {
		logString("[lsm9ds1,MAG:Init] > Success\n");
	} else {
		logString("[lsm9ds1,MAG:Init] > Failed\n");
		return 0;
	}

	return status;
}

uint8_t lsm9ds1AGTCheckConnection() {
	uint8_t status = spi3Read(LSM9DS1_WHO_AM_I_XG, lsm9ds1.buffer, 1, LSM9DS1_AG_DEVICE);
	if (status) {
		status = (lsm9ds1.buffer[0] == LSM9DS1_WHO_AM_I_AG_RSP);
	}
	return status;
}

uint8_t lsm9ds1MCheckConnection() {
	uint8_t status = spi3Read(LSM9DS1_WHO_AM_I_M, lsm9ds1.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (status) {
		status = (lsm9ds1.buffer[0] == LSM9DS1_WHO_AM_I_M_RSP);
	}
	return status;
}

float getLsm9ds1MaxValidG() {
	return lsm9ds1.maxValidG * 0.85f;
}


void lsm9ds1DeriveGyroSensitivity() {
	switch (lsm9ds1.gyroFullScale) {
	case LSM9DS1_GYRORANGE_245DPS:
		lsm9ds1.gyroSensitivity = 245.0f / 32768.0f;
		lsm9ds1.maxValidRotationRate = 245.0f;
		break;
	case LSM9DS1_GYRORANGE_500DPS:
		lsm9ds1.gyroSensitivity = 500.0f / 32768.0f;
		lsm9ds1.maxValidRotationRate = 500.0f;
		break;
	case LSM9DS1_GYRORANGE_2000DPS:
		lsm9ds1.gyroSensitivity = 2000.0f / 32768.0f;
		lsm9ds1.maxValidRotationRate = 2000.0f;
		break;
	}
}

void lsm9ds1DeriveAccSensitivity() {
	switch (lsm9ds1.accFullScale) {
	case LSM9DS1_ACCELRANGE_2G:
		lsm9ds1.accSensitivity = 2.0f / 32768.0f;
		lsm9ds1.maxValidG = 2.0f;
		break;
	case LSM9DS1_ACCELRANGE_4G:
		lsm9ds1.accSensitivity = 4.0f / 32768.0f;
		lsm9ds1.maxValidG = 4.0f;
		break;
	case LSM9DS1_ACCELRANGE_8G:
		lsm9ds1.accSensitivity = 8.0f / 32768.0f;
		lsm9ds1.maxValidG = 8.0f;
		break;
	case LSM9DS1_ACCELRANGE_16G:
		lsm9ds1.accSensitivity = 16.0f / 32768.0f;
		lsm9ds1.maxValidG = 16.0f;
		break;
	}
}

void lsm9ds1DeriveMagSensitivity() {
	switch (lsm9ds1.magFullScale) {
	case LSM9DS1_MAGRANGE_4GAUSS:
		lsm9ds1.magSensitivity = 0.00014f;
		break;
	case LSM9DS1_MAGRANGE_8GAUSS:
		lsm9ds1.magSensitivity = 0.00029f;
		break;
	case LSM9DS1_MAGRANGE_16GAUSS:
		lsm9ds1.magSensitivity = 0.00058f;
		break;
	}
}

uint8_t lsm9ds1AGInit(int16_t pGyroFullScale, int16_t pAccFullScale) {
	uint8_t status = 0;

	lsm9ds1.gyroFullScale = pGyroFullScale;
	lsm9ds1.accFullScale = pAccFullScale;

	//Reset and allow burst reads
	lsm9ds1.buffer[0] = 0x1; //00000001
	status = spi3Write(LSM9DS1_CTRL_REG8, lsm9ds1.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Gyro ODR : 952 , FS:245 , Filter : 100 Hz
	//110 00 0 11
	lsm9ds1.buffer[0] = lsm9ds1.gyroFullScale;
	status = spi3Write(LSM9DS1_CTRL_REG1_G, lsm9ds1.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Disable Fifo
	lsm9ds1.buffer[0] = 0x0;
	status = spi3Write(LSM9DS1_FIFO_CTRL, lsm9ds1.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//0000[int 0:1][out sel 0:1]
	//0000 00 00
	lsm9ds1.buffer[0] = 0x0;
	status = spi3Write(LSM9DS1_CTRL_REG2_G, lsm9ds1.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//LP_mode HP_EN 0(1) 0(1) HPCF3_G HPCF2_G HPCF1_G HPCF0_G
	//0 0 00 1001
	lsm9ds1.buffer[0] = 0x9;
	status = spi3Write(LSM9DS1_CTRL_REG3_G, lsm9ds1.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Enable XYZ accelerometer no decimation
	lsm9ds1.buffer[0] = 0x38;
	status = spi3Write(LSM9DS1_CTRL_REG5_XL, lsm9ds1.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	lsm9ds1.buffer[0] = lsm9ds1.accFullScale;
	status = spi3Write(LSM9DS1_CTRL_REG6_XL, lsm9ds1.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//HR DCF1 DCF0 0(1) 0(1) FDS 0(1) HPIS1
	//1 10 0 00 0 0
	lsm9ds1.buffer[0] = 0xC0;
	status = spi3Write(LSM9DS1_CTRL_REG7_XL, lsm9ds1.buffer, 1, LSM9DS1_AG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	lsm9ds1.tempSensitivity = 1.0 / 16.0f;
	lsm9ds1DeriveGyroSensitivity();
	lsm9ds1DeriveAccSensitivity();

	return status;
}

uint8_t lsm9ds1MInit(int16_t pMagFullScale) {
	uint8_t status = 0;
	lsm9ds1.magFullScale = pMagFullScale;
	//Soft Reset
	lsm9ds1.buffer[0] = 0x4;
	status = spi3Write(LSM9DS1_CTRL_REG2_M, lsm9ds1.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Temp compensation enabled , high perf , ODR : 80Hz , Fast mode
	lsm9ds1.buffer[0] = 0xDE;
	status = spi3Write(LSM9DS1_CTRL_REG1_M, lsm9ds1.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	lsm9ds1.buffer[0] = pMagFullScale;
	status = spi3Write(LSM9DS1_CTRL_REG2_M, lsm9ds1.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//I2C disable, no low power , continuous mode
	lsm9ds1.buffer[0] = 0x80;
	status = spi3Write(LSM9DS1_CTRL_REG3_M, lsm9ds1.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Z axis High performance mode
	lsm9ds1.buffer[0] = 0x8;
	status = spi3Write(LSM9DS1_CTRL_REG4_M, lsm9ds1.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	delayMs(10);

	//Fast read disabled , continuous mode
	lsm9ds1.buffer[0] = 0x0;
	status = spi3Write(LSM9DS1_CTRL_REG5_M, lsm9ds1.buffer, 1, LSM9DS1_MAG_DEVICE);
	if (!status) {
		return 0;
	}
	lsm9ds1DeriveMagSensitivity();
	return status;
}

void lsm9ds1ApplyAccOffsetCorrection() {
	lsm9ds1.rawAx -= lsm9ds1.offsetAx;
	lsm9ds1.rawAy -= lsm9ds1.offsetAy;
	lsm9ds1.rawAz -= lsm9ds1.offsetAz;
}

void lsm9ds1ApplyGyroOffsetCorrection() {
	lsm9ds1.rawGx -= lsm9ds1.offsetGx;
	lsm9ds1.rawGy -= lsm9ds1.offsetGy;
	lsm9ds1.rawGz -= lsm9ds1.offsetGz;
}

void lsm9ds1ApplyAccDataScaling() {
	lsm9ds1.axG = (float) lsm9ds1.rawAx * lsm9ds1.accSensitivity;
	lsm9ds1.ayG = (float) lsm9ds1.rawAy * lsm9ds1.accSensitivity;
	lsm9ds1.azG = (float) lsm9ds1.rawAz * lsm9ds1.accSensitivity;
}

void lsm9ds1ApplyGyroDataScaling() {
	lsm9ds1.gxDS = (float) lsm9ds1.rawGx * lsm9ds1.gyroSensitivity;
	lsm9ds1.gyDS = (float) lsm9ds1.rawGy * lsm9ds1.gyroSensitivity;
	lsm9ds1.gzDS = (float) lsm9ds1.rawGz * lsm9ds1.gyroSensitivity;
}

void lsm9ds1ApplyMagDataScaling() {
	lsm9ds1.mx = ((float) lsm9ds1.rawMx * lsm9ds1.magSensitivity);
	lsm9ds1.my = ((float) lsm9ds1.rawMy * lsm9ds1.magSensitivity);
	lsm9ds1.mz = ((float) lsm9ds1.rawMz * lsm9ds1.magSensitivity);
}

void lsm9ds1ApplyAccTempOffsetCorrection() {
#if LSM9DS1_APPLY_ACC_TEMP_OFFSET_CORRECTION ==1
	lsm9ds1.axG -= lsm9ds1.accXTempOffset;
	lsm9ds1.ayG -= lsm9ds1.accYTempOffset;
	lsm9ds1.azG -= lsm9ds1.accZTempOffset;
#endif
}

void lsm9ds1ApplyGyroTempOffsetCorrection() {
#if LSM9DS1_APPLY_GYRO_TEMP_OFFSET_CORRECTION ==1
	lsm9ds1.gxDS -= lsm9ds1.gyroXTempOffset;
	lsm9ds1.gyDS -= lsm9ds1.gyroYTempOffset;
	lsm9ds1.gzDS -= lsm9ds1.gyroZTempOffset;
#endif
}

void lsm9ds1ApplyMagOffsetCorrection() {
	/*
	 lsm9ds1.mx *= lsm9ds1.offsetMx;
	 lsm9ds1.my *= lsm9ds1.offsetMy;
	 lsm9ds1.mz *= lsm9ds1.offsetMz;
	*/
	//Apply the bias if already set
	lsm9ds1.mx -= lsm9ds1.biasMx;
	lsm9ds1.my -= lsm9ds1.biasMy;
	lsm9ds1.mz -= lsm9ds1.biasMz;
	//Apply the scales for each axis
	lsm9ds1.mx *= lsm9ds1.scaleMx;
	lsm9ds1.my *= lsm9ds1.scaleMy;
	lsm9ds1.mz *= lsm9ds1.scaleMz;
}

void lsm9ds1ReadGyro() {
	if (spi3Read(LSM9DS1_OUT_X_L_G, lsm9ds1.buffer, 6, LSM9DS1_AG_DEVICE)) {
		lsm9ds1.rawGx = (((int16_t) lsm9ds1.buffer[1]) << 8) | (lsm9ds1.buffer[0] & 0xFF);
		lsm9ds1.rawGy = (((int16_t) lsm9ds1.buffer[3]) << 8) | (lsm9ds1.buffer[2] & 0xFF);
		lsm9ds1.rawGz = (((int16_t) lsm9ds1.buffer[5]) << 8) | (lsm9ds1.buffer[4] & 0xFF);
	}
}

void lsm9ds1ReadAcc() {
	if (spi3Read(LSM9DS1_OUT_X_L_XL, lsm9ds1.buffer, 6, LSM9DS1_AG_DEVICE)) {
		lsm9ds1.rawAx = (((int16_t) lsm9ds1.buffer[1]) << 8) | (lsm9ds1.buffer[0] & 0xFF);
		lsm9ds1.rawAy = (((int16_t) lsm9ds1.buffer[3]) << 8) | (lsm9ds1.buffer[2] & 0xFF);
		lsm9ds1.rawAz = (((int16_t) lsm9ds1.buffer[5]) << 8) | (lsm9ds1.buffer[4] & 0xFF);
	}
}

void lsm9ds1ReadAccAndGyro() {
	if (spi3Read(LSM9DS1_OUT_X_L_G, lsm9ds1.buffer, 12, LSM9DS1_AG_DEVICE)) {
		lsm9ds1.rawGx = (((int16_t) lsm9ds1.buffer[1]) << 8)  | (lsm9ds1.buffer[0] & 0xFF);
		lsm9ds1.rawGy = (((int16_t) lsm9ds1.buffer[3]) << 8)  | (lsm9ds1.buffer[2] & 0xFF);
		lsm9ds1.rawGz = (((int16_t) lsm9ds1.buffer[5]) << 8)  | (lsm9ds1.buffer[4] & 0xFF);
		lsm9ds1.rawAx = (((int16_t) lsm9ds1.buffer[7]) << 8)  | (lsm9ds1.buffer[6] & 0xFF);
		lsm9ds1.rawAy = (((int16_t) lsm9ds1.buffer[9]) << 8)  | (lsm9ds1.buffer[8] & 0xFF);
		lsm9ds1.rawAz = (((int16_t) lsm9ds1.buffer[11]) << 8) | (lsm9ds1.buffer[10] & 0xFF);
	}
}

void lsm9ds1ReadTemp() {
	if (spi3Read(LSM9DS1_OUT_TEMP_L, lsm9ds1.buffer, 2, LSM9DS1_AG_DEVICE)) {
		lsm9ds1.rawTemp = (((int16_t) lsm9ds1.buffer[1]) << 8) | (lsm9ds1.buffer[0] & 0xFF);
		lsm9ds1.tempC = (float) lsm9ds1.rawTemp * lsm9ds1.tempSensitivity + 25.0f;
	}
}

void lsm9ds1ReadMag() {
	if (spi3Read((LSM9DS1_OUT_READ_MULTIPLE | LSM9DS1_OUT_X_L_M), lsm9ds1.buffer, 6, LSM9DS1_MAG_DEVICE)) {
		lsm9ds1.rawMx = (((int16_t) lsm9ds1.buffer[1]) << 8) | (lsm9ds1.buffer[0] & 0xFF);
		lsm9ds1.rawMy = (((int16_t) lsm9ds1.buffer[3]) << 8) | (lsm9ds1.buffer[2] & 0xFF);
		lsm9ds1.rawMz = (((int16_t) lsm9ds1.buffer[5]) << 8) | (lsm9ds1.buffer[4] & 0xFF);
	}
}

