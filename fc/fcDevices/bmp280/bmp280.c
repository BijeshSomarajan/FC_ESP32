#include "../../devices/bmp280/bmp280.h"

uint8_t BMP280Posr      = P_OSR_16;
uint8_t BMP280Tosr      = T_OSR_02;
uint8_t BMP280Mode      = Normal;
uint8_t BMP280IIRFilter = FC_16 ;
uint8_t BMP280SBy       = t_00_5ms;

uint8_t bmp280CheckConnection() {
	uint8_t buffer[1];
	bmp280ReadRegister(BMP280_WHO_AM_I, buffer, 1);
	if (buffer[0] == BMP280_WHO_AM_I_RETURN) {
		return 1;
	} else {
		return 0;
	}
}

void bmp280Reset() {
	uint8_t buffer[1];
	buffer[0] = 0xB6;
	bmp280WriteRegister(BMP280_RESET, buffer, 1);
}

uint8_t bmp280Init() {
	uint8_t buffer[24];
	//Soft reset
	buffer[0] = 0xB6;
	bmp280WriteRegister(BMP280_RESET, buffer, 1);
	delayMs(100);
	//Check the status
	bmp280ReadRegister(BMP280_STATUS, buffer, 1);
	while ((buffer[0] && 0x01) == 0x01) {
		bmp280ReadRegister(BMP280_STATUS, buffer, 1);
	}
	delayMs(100);
	// Read and store calibration
	buffer[0] = 0;
	bmp280ReadRegister(BMP280_CALIB00, buffer, 24);
	bmp280.dig_T1 = (uint16_t)(((uint16_t) buffer[1] << 8) | buffer[0]);
	bmp280.dig_T2 = (int16_t)(((int16_t) buffer[3] << 8) | buffer[2]);
	bmp280.dig_T3 = (int16_t)(((int16_t) buffer[5] << 8) | buffer[4]);
	bmp280.dig_P1 = (uint16_t)(((uint16_t) buffer[7] << 8) | buffer[6]);
	bmp280.dig_P2 = (int16_t)(((int16_t) buffer[9] << 8) | buffer[8]);
	bmp280.dig_P3 = (int16_t)(((int16_t) buffer[11] << 8) | buffer[10]);
	bmp280.dig_P4 = (int16_t)(((int16_t) buffer[13] << 8) | buffer[12]);
	bmp280.dig_P5 = (int16_t)(((int16_t) buffer[15] << 8) | buffer[14]);
	bmp280.dig_P6 = (int16_t)(((int16_t) buffer[17] << 8) | buffer[16]);
	bmp280.dig_P7 = (int16_t)(((int16_t) buffer[19] << 8) | buffer[18]);
	bmp280.dig_P8 = (int16_t)(((int16_t) buffer[21] << 8) | buffer[20]);
	bmp280.dig_P9 = (int16_t)(((int16_t) buffer[23] << 8) | buffer[22]);

	// Set standby time interval in normal mode and bandwidth
	buffer[0] = (BMP280SBy << 5) | (BMP280IIRFilter << 2);
	if (bmp280WriteRegister(BMP280_CONFIG, buffer, 1) == 1) {
		delayMs(100);
		// Set T and P oversampling rates and sensor mode
		buffer[0] = (BMP280Tosr << 5) | (BMP280Posr << 2) | BMP280Mode;
		if (bmp280WriteRegister(BMP280_CTRL_MEAS, buffer, 1) == 1) {
			delayMs(100);
			bmp280.temperature = 0;
			bmp280.pressure = 0;
			bmp280.altitude = 0;
		}
	}
	return 1;
}

void bmp280Read() {
	uint8_t rawData[6];
	bmp280ReadRegister(BMP280_PRESS_MSB, rawData, 6);
	bmp280.rawPressure = (int32_t)(((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
	bmp280.rawTemperature = (int32_t)(((int32_t) rawData[3] << 16 | (int32_t) rawData[4] << 8 | rawData[5]) >> 4);
}

void bmp280UpdateAltitude() {
	bmp280.altitude = 44330.0f * (1.0f - powf((bmp280.pressure / 1014.0f), 0.1903f));
	//bmp280.altitude = 145366.45f * (1.0f - pow((bmp280.pressure / 1013.25f), 0.1903f));
}

void bmp280UpdateTemperature() {
	int32_t var1, var2, T;
	int32_t adc_T = bmp280.rawTemperature;
	var1 = ((((adc_T >> 3) - ((int32_t) bmp280.dig_T1 << 1))) * ((int32_t) bmp280.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t) bmp280.dig_T1)) * ((adc_T >> 4) - ((int32_t) bmp280.dig_T1))) >> 12) * ((int32_t) bmp280.dig_T3)) >> 14;
	bmp280.t_fine = var1 + var2;
	T = (bmp280.t_fine * 5 + 128) >> 8;
	bmp280.temperature = T / 100.0f;
}

void bmp280UpdatePressure() {
	long long var1, var2, p;
	int32_t adc_P = bmp280.rawPressure;
	var1 = ((int64_t) bmp280.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t) bmp280.dig_P6;
	var2 = var2 + ((var1 * (int64_t) bmp280.dig_P5) << 17);
	var2 = var2 + (((int64_t) bmp280.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) bmp280.dig_P3) >> 8) + ((var1 * (int64_t) bmp280.dig_P2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) bmp280.dig_P1) >> 33;
	if (var1 == 0) {
		bmp280.pressure = 0;
	} else {
		p = 1048576 - adc_P;
		p = (((p << 31) - var2) * 3125) / var1;
		var1 = (((int64_t) bmp280.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		var2 = (((int64_t) bmp280.dig_P8) * p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((int64_t) bmp280.dig_P7) << 4);
		bmp280.pressure = (float) p / 25600.0f;
	}
}

