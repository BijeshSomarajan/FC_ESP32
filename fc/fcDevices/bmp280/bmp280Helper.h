#ifndef CORE_DEVICES_BMP280_BMP280HELPER_H_
#define CORE_DEVICES_BMP280_BMP280HELPER_H_
#include <string.h>
#include <stdint.h>
#include "../../io/spi/spi.h"

int bmp280ReadRegister(unsigned char regAddr, unsigned char* readBuffer, int bytes);
int bmp280WriteRegister(unsigned char regAddr, unsigned char* writeBuffer, int bytes);

#endif /* CORE_DEVICES_BMP280_BMP280HELPER_H_ */
