#ifndef CORE_DEVICES_BMP280_BMP280_H_
#define CORE_DEVICES_BMP280_BMP280_H_

#include <string.h>
#include <stdint.h>
#include <math.h>

#include "../../devices/bmp280/bmp280Helper.h"
#include "../../devices/bmp280/bmp280Registers.h"
#include "../../timer/delay/delayTimer.h"

typedef struct _BMP280 BMP280;
struct _BMP280 {
	int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	uint16_t dig_T1, dig_P1;
	int32_t rawTemperature, rawPressure;
	float temperature, pressure, altitude;
	int32_t t_fine;
};
BMP280 bmp280;

uint8_t bmp280Init(void);
uint8_t bmp280CheckConnection(void);
void bmp280Reset();
void bmp280Read(void);
void bmp280UpdateAltitude(void);
void bmp280UpdateTemperature(void);
void bmp280UpdatePressure(void);

#endif /* CORE_DEVICES_BMP280_BMP280_H_ */
