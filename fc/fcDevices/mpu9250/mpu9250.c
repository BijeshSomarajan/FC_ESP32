#include "memsCommon.h"
#if MEMS_SENSOR_SELECTED == MEMS_SENSOR_MPU9250
#include "mpu9250Helper.h"
#include "mpu9250Registers.h"
#include "delayTimer.h"
#include "fcLogger.h"
#include "mathUtil.h"

uint8_t mpu9250CheckConnection(void);
uint8_t ak8963CheckConnection(void);
void mpu9250Init(int16_t pGyroFullScale, int16_t pAccFullScale, int16_t pMagFullScale);
uint8_t mpu9250Open(void);
uint8_t ak8963Open(void);
void mpu9250Reset(void);

extern MEMSDATA memsData;

uint8_t memsMInit() {
	uint8_t status = initSPI3();
	if (status) {
		logString("[mpu9250,IO:SPI-3] > Success\n");
	} else {
		logString("[mpu9250,IO:SPI-3] > Failed\n");
		return 0;
	}
	status = mpu9250CheckConnection();
	if (status) {
		logString("[mpu9250,AGT:CON] > Success\n");
	} else {
		logString("[mpu9250,AGT:CON] > Failed\n");
		return 0;
	}
	status = ak8963CheckConnection();
	if (status) {
		logString("[ak8963,MAG:CON] > Success\n");
	} else {
		logString("[ak8963,MAG:CON] > Failed\n");
		return 0;
	}
	mpu9250Init(MPU9250_GYRO_FS_250, MPU9250_ACCEL_FS_4, MPU9250_MAG_FS_16BITS);
	status = mpu9250Open();
	if (status) {
		logString("[mpu9250,AGT:Init] > Success\n");
	} else {
		logString("[mpu9250,AGT:Init] > Failed\n");
		return 0;
	}
	status = ak8963Open();
	if (status) {
		logString("[ak8963,MAG:Init] > Success\n");
	} else {
		logString("[ak8963,MAG:Init] > Failed\n");
		return 0;
	}
	return status;
}

uint8_t mpu9250CheckConnection() {
	memsData.buffer[0] = 0;
	mpu9250ReadByte(MPU9250_WHO_AM_I, memsData.buffer);
	if (memsData.buffer[0] == MPU9250_WHO_AM_I_RETURN || memsData.buffer[0] == MPU9255_WHO_AM_I_RETURN) {
		return 1;
	} else {
		return 0;
	}
}

uint8_t ak8963CheckConnection() {
	mpu9250WriteByte(I2C_SLV0_ADDR, AK8963_ADDRESS | READ_FLAG); //Set i2C slave address of AK8963 and set for read
	mpu9250WriteByte(I2C_SLV0_REG, AK8963_WHO_AM_I); //i2C slave 0 register from where to begin data transfer
	mpu9250WriteByte(I2C_SLV0_CTRL, READ_FLAG | 1); //Read one byte from AK8963
	delayMs(200);
	mpu9250ReadByte(EXT_SENS_DATA_00, memsData.buffer); //Read i2C
	if (memsData.buffer[0] == AK8963_WHO_AM_I_RETURN) {
		return 1;
	} else {
		return 0;
	}
}

void ak8963Reset() {
	mpu9250WriteByte(I2C_SLV0_DO, 0x1);
	delayMs(200);
}

uint8_t ak8963Open() {
	ak8963Reset();
	mpu9250WriteByte( I2C_SLV0_ADDR, AK8963_ADDRESS); //Set i2C slave0 address as AK8963
	mpu9250WriteByte( I2C_SLV0_REG, AK8963_CNTL2); //i2c slave 0 register address for where to begin data transfer
	mpu9250WriteByte( I2C_SLV0_DO, 0x1); //Reset AK8963
	mpu9250WriteByte( I2C_SLV0_CTRL, READ_FLAG | 1); //Enable i2c and set 1 byte
	mpu9250WriteByte( I2C_SLV0_REG, AK8963_CNTL1); //Enable i2c and set 1 byte
	mpu9250WriteByte( I2C_SLV0_DO, 0x16); //Reset AK8963
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	mpu9250WriteByte( AK8963_CNTL1, memsData.magFullScale << 4 | AK8963_MODE); // Set magnetometer data resolution and sample ODR
	delayMs(1000);
	return 1;
}

void disableMPU9250Sleep() {
	mpu9250WriteByte( PWR_MGMT_1, 0);
	delayMs(200);
}

void enableMPU9250Sleep() {
	mpu9250WriteByte( PWR_MGMT_1, 1);
	delayMs(200);
}

uint8_t mpu9250Open() {
	ak8963Reset();
	mpu9250Reset();
	disableMPU9250Sleep();
	delayMs(200);
	//Set clock source ,It is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source) as the clock reference for improved stability
	mpu9250WriteBits( PWR_MGMT_1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, CLOCK_PLL_ZGYRO);
	// Configure Gyro and Accelerometer Low Pass filters
	mpu9250WriteBits( CONFIG, CFG_DLPF_CFG_BIT, CFG_DLPF_CFG_LENGTH, DLPF_BW_256);
	delayMs(200);
	//set sample rate
	//ACC : 1khz / (1 + x), GYRO : 8khz/(1+x) (if LDP256)
	mpu9250WriteByte( SMPLRT_DIV, 0);
	delayMs(200);
	//set gyroscope range
	mpu9250WriteBits( GYRO_CONFIG, GCONFIG_FS_SEL_BIT, GCONFIG_FS_SEL_LENGTH, memsData.gyroFullScale);
	delayMs(200);
	mpu9250WriteBits( ACCEL_CONFIG, ACONFIG_AFS_SEL_BIT, ACONFIG_AFS_SEL_LENGTH, memsData.accFullScale);
	delayMs(200);

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the master
	mpu9250WriteByte( INT_PIN_CFG, 0x12); //Set i2C byepass
	mpu9250WriteByte( USER_CTRL, 0x30); //i2C master mode , disable slave mode
	mpu9250WriteByte( I2C_MST_CTRL, 0x0D); //i2c mater config at 400Khz
	delayMs(200);
	mpu9250WriteByte( INT_ENABLE, 0x00);   // Disable all interrupts
	delayMs(200);
	mpu9250WriteByte( FIFO_EN, 0x00);      // Disable FIFO
	delayMs(200);
	mpu9250WriteByte( PWR_MGMT_1, 0x01);   // Turn on internal clock source
	delayMs(200);
	return 1;
}

void mpu9250Reset() {
	// Write a one to bit 7 reset bit; toggle reset device
	mpu9250WriteByte( PWR_MGMT_1, 0x80);
	delayMs(500);
	mpu9250WriteByte( PWR_MGMT_1, 0x01);   // Turn on internal clock source
	delayMs(1000);
}

void deriveMPU9250GyroSensitivity() {
	switch (memsData.gyroFullScale) {
	case MPU9250_GYRO_FS_250:
		memsData.gyroSensitivity = 250.0f / 32768.0f;
		memsData.maxValidRotationRate = 250.0f;
		break;
	case MPU9250_GYRO_FS_500:
		memsData.gyroSensitivity = 500.0f / 32768.0f;
		memsData.maxValidRotationRate = 500.0f;
		break;
	case MPU9250_GYRO_FS_1000:
		memsData.gyroSensitivity = 1000.0f / 32768.0f;
		memsData.maxValidRotationRate = 1000.0f;
		break;
	case MPU9250_GYRO_FS_2000:
		memsData.gyroSensitivity = 2000.0f / 32768.0f;
		memsData.maxValidRotationRate = 2000.0f;
		break;
	}
}

void deriveMPU9250AccSensitivity() {
	switch (memsData.accFullScale) {
	case MPU9250_ACCEL_FS_2:
		memsData.accSensitivity = 2.0f / 32768.0f;
		memsData.maxValidG = 2.0f;
		break;
	case MPU9250_ACCEL_FS_4:
		memsData.accSensitivity = 4.0f / 32768.0f;
		memsData.maxValidG = 4.0f;
		break;
	case MPU9250_ACCEL_FS_8:
		memsData.accSensitivity = 8.0f / 32768.0f;
		memsData.maxValidG = 8.0f;
		break;
	case MPU9250_ACCEL_FS_16:
		memsData.accSensitivity = 16.0f / 32768.0f;
		memsData.maxValidG = 16.0f;
		break;
	}
}

void deriveMPU9250MagSensitivity() {
	switch (memsData.magFullScale) {
	case MPU9250_MAG_FS_14BITS:
		memsData.magSensitivity = 10.0f * 4912.0 / 8190.0;
		break;
	case MPU9250_MAG_FS_16BITS:
		memsData.magSensitivity = 10.0f * 4912.0 / 32760.0;
		break;
	}
}

void mpu9250ResetData() {
	memsData.rawAx = 0;
	memsData.rawAy = 0;
	memsData.rawAz = 0;
	memsData.rawGx = 0;
	memsData.rawGy = 0;
	memsData.rawGz = 0;
	memsData.rawMx = 0;
	memsData.rawMy = 0;
	memsData.rawMz = 0;
}

void mpu9250Init(int16_t pGyroFullScale, int16_t pAccFullScale, int16_t pMagFullScale) {
	memsData.accFullScale = pAccFullScale;
	memsData.gyroFullScale = pGyroFullScale;
	memsData.magFullScale = pMagFullScale;
	//Determine the resolutions
	deriveMPU9250GyroSensitivity();
	deriveMPU9250AccSensitivity();
	deriveMPU9250MagSensitivity();
	//Initialize the variables
	memsData.offsetAx = 0;
	memsData.offsetAy = 0;
	memsData.offsetAz = 0;

	memsData.offsetGx = 0;
	memsData.offsetGy = 0;
	memsData.offsetGz = 0;

	memsData.offsetMx = 0;
	memsData.offsetMy = 0;
	memsData.offsetMz = 0;

	memsData.biasMx = 0;
	memsData.biasMy = 0;
	memsData.biasMz = 0;

	memsData.scaleMx = 0;
	memsData.scaleMy = 0;
	memsData.scaleMz = 0;

	memsData.accXTempOffset = 0;
	memsData.accXTempCoeff[0] = 0;
	memsData.accXTempCoeff[1] = 0;
	memsData.accXTempCoeff[2] = 0;
	memsData.accXTempCoeff[3] = 0;

	memsData.accYTempOffset = 0;
	memsData.accYTempCoeff[0] = 0;
	memsData.accYTempCoeff[1] = 0;
	memsData.accYTempCoeff[2] = 0;
	memsData.accYTempCoeff[3] = 0;

	memsData.accZTempOffset = 0;
	memsData.accZTempCoeff[0] = 0;
	memsData.accZTempCoeff[1] = 0;
	memsData.accZTempCoeff[2] = 0;
	memsData.accZTempCoeff[3] = 0;

	memsData.gyroXTempOffset = 0;
	memsData.gyroXTempCoeff[0] = 0;
	memsData.gyroXTempCoeff[1] = 0;
	memsData.gyroXTempCoeff[2] = 0;
	memsData.gyroXTempCoeff[3] = 0;

	memsData.gyroYTempOffset = 0;
	memsData.gyroYTempCoeff[0] = 0;
	memsData.gyroYTempCoeff[1] = 0;
	memsData.gyroYTempCoeff[2] = 0;
	memsData.gyroYTempCoeff[3] = 0;

	memsData.gyroZTempOffset = 0;
	memsData.gyroZTempCoeff[0] = 0;
	memsData.gyroZTempCoeff[1] = 0;
	memsData.gyroZTempCoeff[2] = 0;
	memsData.gyroZTempCoeff[3] = 0;
}

void memsReadAcc() {
	mpu9250ReadBytes( ACCEL_XOUT_H, 6, memsData.buffer);  // Read the six raw data registers into data array
	memsData.rawAx = convertBytesToInt16(memsData.buffer[0], memsData.buffer[1]);
	memsData.rawAy = convertBytesToInt16(memsData.buffer[2], memsData.buffer[3]);
	memsData.rawAz = convertBytesToInt16(memsData.buffer[4], memsData.buffer[5]);
}

void memsReadGyro() {
	mpu9250ReadBytes( GYRO_XOUT_H, 6, &memsData.buffer[0]);
	memsData.rawGx = convertBytesToInt16(memsData.buffer[0], memsData.buffer[1]);
	memsData.rawGy = convertBytesToInt16(memsData.buffer[2], memsData.buffer[3]);
	memsData.rawGz = convertBytesToInt16(memsData.buffer[4], memsData.buffer[5]);
}

void memsReadTemp() {
	mpu9250ReadBytes( TEMP_OUT_H, 2, memsData.buffer);
	memsData.rawTemp = convertBytesToInt16(memsData.buffer[0], memsData.buffer[1]);
}

void memsReadAccAndGyro() {
	mpu9250ReadBytes( ACCEL_XOUT_H, 15, &memsData.buffer[0]);  // Read the six raw data registers into data array
	memsData.rawAx = convertBytesToInt16(memsData.buffer[0], memsData.buffer[1]);
	memsData.rawAy = convertBytesToInt16(memsData.buffer[2], memsData.buffer[3]);
	memsData.rawAz = convertBytesToInt16(memsData.buffer[4], memsData.buffer[5]);
	memsData.rawTemp = convertBytesToInt16(memsData.buffer[6], memsData.buffer[7]);
	memsData.rawGx = convertBytesToInt16(memsData.buffer[8], memsData.buffer[9]);
	memsData.rawGy = convertBytesToInt16(memsData.buffer[10], memsData.buffer[11]);
	memsData.rawGz = convertBytesToInt16(memsData.buffer[12], memsData.buffer[13]);
}

void memsReadMag() {
	mpu9250WriteByte( I2C_SLV0_ADDR, AK8963_ADDRESS | READ_FLAG); //Set i2C slave0 address as AK8963 in read mode
	mpu9250WriteByte( I2C_SLV0_REG, AK8963_XOUT_L); //i2c slave 0 register address for where to begin data transfer
	mpu9250WriteByte( I2C_SLV0_CTRL, READ_FLAG | 7); //Enable i2c and read 7 bytes
	mpu9250ReadBytes( EXT_SENS_DATA_00, 7, memsData.buffer);  // Read the six raw data and ST2 registers sequentially into data array
	uint8_t c = memsData.buffer[6]; // End data read by reading ST2 register
	if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
		memsData.rawMx = (int16_t) (((int16_t) memsData.buffer[1] << 8) | memsData.buffer[0]);  // Turn the MSB and LSB into a signed 16-bit value
		memsData.rawMy = (int16_t) (((int16_t) memsData.buffer[3] << 8) | memsData.buffer[2]); // Data stored as little Endian
		memsData.rawMz = (int16_t) (((int16_t) memsData.buffer[5] << 8) | memsData.buffer[4]);
	}
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

void memsApplyMagOffsetCorrection() {
	//Apply the offsets if already set
	memsData.mx *= memsData.offsetMx;
	memsData.my *= memsData.offsetMy;
	memsData.mz *= memsData.offsetMz;

	//Apply the bias if already set
	memsData.mx -= memsData.biasMx;
	memsData.my -= memsData.biasMy;
	memsData.mz -= memsData.biasMz;

	//Apply the scales for each axis
	memsData.mx *= memsData.scaleMx;
	memsData.my *= memsData.scaleMy;
	memsData.mz *= memsData.scaleMz;
}

void memsApplyAccTempOffsetCorrection() {
#if MPU9250_APPLY_ACC_TEMP_OFFSET_CORRECTION ==1
	memsData.axG -= memsData.accXTempOffset;
	memsData.ayG -= memsData.accYTempOffset;
	memsData.azG -= memsData.accZTempOffset;
#endif
}

void memsApplyGyroTempOffsetCorrection() {
#if MPU9250_APPLY_GYRO_TEMP_OFFSET_CORRECTION ==1
	memsData.gxDS -= memsData.gyroXTempOffset;
	memsData.gyDS -= memsData.gyroYTempOffset;
	memsData.gzDS -= memsData.gyroZTempOffset;
#endif
}

void memsApplyAccDataScaling() {
	memsData.axG = memsData.rawAx * memsData.accSensitivity;
	memsData.ayG = memsData.rawAy * memsData.accSensitivity;
	memsData.azG = memsData.rawAz * memsData.accSensitivity;
}

void memsApplyGyroDataScaling() {
	memsData.gxDS = memsData.rawGx * memsData.gyroSensitivity;
	memsData.gyDS = memsData.rawGy * memsData.gyroSensitivity;
	memsData.gzDS = memsData.rawGz * memsData.gyroSensitivity;
}

void memsApplyTempDataScaling() {
	memsData.tempC = (memsData.rawTemp / 333.87f);
}

void memsApplyMagDataScaling() {
	memsData.mx = ((float) memsData.rawMx * memsData.magSensitivity);
	memsData.my = ((float) memsData.rawMy * memsData.magSensitivity);
	memsData.mz = ((float) memsData.rawMz * memsData.magSensitivity);
}

float getMemsMaxValidG() {
	return memsData.maxValidG * 0.85f;
}

#endif
