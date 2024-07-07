#include "mpu9250Helper.h"

#define MPU9250_AG_DEVICE SPI3_DEVICE_1

/*********************************************************************************************************/
/* Reads 'N' number of Bytes from the device register to the data pointer , returns number of Bytes Read */
/*********************************************************************************************************/
uint8_t mpu9250ReadBytes(uint8_t regAddr, int32_t length, uint8_t *data) {
	if (spi3Read(regAddr, data, length, MPU9250_AG_DEVICE)) {
		return length;
	}
	return 0;
}

/***********************************************************************************************/
/* Writes 'N' number of bytes to the device register provided  via data pointer                */
/* Note this function is not thread safe                                                       */
/***********************************************************************************************/
uint8_t mpu9250WriteBytes(uint8_t regAddr, int_fast32_t length, uint8_t *data) {
	if (spi3Write(regAddr, data, length, MPU9250_AG_DEVICE)) {
		return length;
	}
	return 0;
}
/******************************************************************************************/
/* Reads 1 Byte from the device register to the pointer , returns 1 on success            */
/******************************************************************************************/
uint8_t mpu9250ReadByte(uint8_t regAddr, uint8_t *data) {
	return mpu9250ReadBytes(regAddr, 1, data);
}
/*********************************************************************************************************************/
/* Reads specific range of Bits in a Byte loaded from the device register to the data pointer,returns number of bits read*/
/*********************************************************************************************************************/
uint8_t mpu9250ReadBits(uint8_t regAddr, uint8_t bitStart, int32_t length, uint8_t *data) {
	uint8_t b;
	if (mpu9250ReadByte(regAddr, &b) == 1) {
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
		return length;
	}
	return 0;
}
/*************************************************************************************************************/
/* Reads a specific bit in a Byte loaded from the device register to the data pointer , returns 1 on success */
/*************************************************************************************************************/
uint8_t mpu9250ReadBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
	uint8_t b;
	if (mpu9250ReadByte(regAddr, &b) == 1) {
		*data = b & (1 << bitNum);
		return 1;
	}
	return 0;
}

/**********************************************************************************************/
/* Writes a singe byte  to the device register                                                */
/**********************************************************************************************/
uint8_t mpu9250WriteByte(uint8_t regAddr, uint8_t data) {
	return mpu9250WriteBytes(regAddr, 1, &data);
}

/*****************************************************************************/
/* Writes the specified number of bits from the data to the device register  */
/*****************************************************************************/
uint8_t mpu9250WriteBits(uint8_t regAddr, uint8_t bitStart, int32_t length, uint8_t data) {
	uint8_t b = 0;
	if (mpu9250ReadByte(regAddr, &b) == 1) {
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		b &= ~(mask); // zero all important bits in existing byte
		b |= data; // combine data with existing byte
		if (mpu9250WriteByte(regAddr, b) == 1) {
			return length;
		}
	}
	return 0;
}

/************************************************************************/
/* Writes a bit from the data to the device register                    */
/************************************************************************/
uint8_t mpu9250WriteBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
	uint8_t b;
	if (mpu9250ReadByte(regAddr, &b) == 1) {
		b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
		if (mpu9250WriteByte(regAddr, b) == 1) {
			return 1;
		}
	}
	return 0;
}

