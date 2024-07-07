#ifndef MPU9250HELPER_H_
#define MPU9250HELPER_H_

#include <string.h>
#include <stdint.h>

#include "fcSPI.h"
#include "mpu9250Registers.h"

uint8_t mpu9250ReadBytes(uint8_t regAddr, int32_t length, uint8_t *data);
uint8_t mpu9250ReadByte(uint8_t regAddr, uint8_t *data);
uint8_t mpu9250ReadBits(uint8_t regAddr, uint8_t bitStart, int32_t length, uint8_t *data);
uint8_t mpu9250ReadBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);
uint8_t mpu9250WriteBytes(uint8_t regAddr, int_fast32_t length, uint8_t *data);
uint8_t mpu9250WriteByte(uint8_t regAddr, uint8_t data);
uint8_t mpu9250WriteBits(uint8_t regAddr, uint8_t bitStart, int32_t length, uint8_t data);
uint8_t mpu9250WriteBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);

#endif
