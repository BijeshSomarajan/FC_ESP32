#include "bmp388.h"
#include "bmp388Registers.h"
#include "fcSPI.h"
#include "delayTimer.h"
#include "fcLogger.h"
#include <math.h>

#define BP388_SPI3_DEVICE 3

BMP388 bmp388;

uint8_t bmp388CheckConnection() {
	uint8_t status = spi3Read(BMP3_CHIP_ID_ADDR, bmp388.buffer, 2, BP388_SPI3_DEVICE);
	if (status) {
		//First byte is dummy
		status = (bmp388.buffer[1] == BMP3_CHIP_ID);
	}
	return status;
}

uint8_t bmpReadCalib() {
	uint8_t status = spi3Read(BMP3_CALIB_DATA_ADDR, bmp388.buffer, BMP3_CALIB_DATA_LEN + 1, BP388_SPI3_DEVICE);
	if (status) {
		bmp388.t1 = BMP3_CONCAT_BYTES(bmp388.buffer[2], bmp388.buffer[1]);
		double temp_var = 0.00390625f; //1/2^8
		bmp388.dt1 = ((double) bmp388.t1 / temp_var);

		bmp388.t2 = BMP3_CONCAT_BYTES(bmp388.buffer[4], bmp388.buffer[3]);
		temp_var = 1073741824.0f; //2^30
		bmp388.dt2 = ((double) bmp388.t2 / temp_var);

		bmp388.t3 = (int8_t) bmp388.buffer[5];
		temp_var = 281474976710656.0f; //2^48
		bmp388.dt3 = ((double) bmp388.t3 / temp_var);

		bmp388.p1 = (int16_t) BMP3_CONCAT_BYTES(bmp388.buffer[7], bmp388.buffer[6]);
		temp_var = 1048576.0f; //2^20
		bmp388.dp1 = ((double) (bmp388.p1 - (16384)) / temp_var);

		bmp388.p2 = (int16_t) BMP3_CONCAT_BYTES(bmp388.buffer[9], bmp388.buffer[8]);
		temp_var = 536870912.0f; //2^29
		bmp388.dp2 = ((double) (bmp388.p2 - (16384)) / temp_var);

		bmp388.p3 = (int8_t) bmp388.buffer[10];
		temp_var = 4294967296.0f; //2^32
		bmp388.dp3 = ((double) bmp388.p3 / temp_var);

		bmp388.p4 = (int8_t) bmp388.buffer[11];
		temp_var = 137438953472.0f; //2^37
		bmp388.dp4 = ((double) bmp388.p4 / temp_var);

		bmp388.p5 = BMP3_CONCAT_BYTES(bmp388.buffer[13], bmp388.buffer[12]);
		temp_var = 0.125f; //1/2^3
		bmp388.dp5 = ((double) bmp388.p5 / temp_var);

		bmp388.p6 = BMP3_CONCAT_BYTES(bmp388.buffer[15], bmp388.buffer[14]);
		temp_var = 64.0f; //2^6
		bmp388.dp6 = ((double) bmp388.p6 / temp_var);

		bmp388.p7 = (int8_t) bmp388.buffer[16];
		temp_var = 256.0f; //2^8
		bmp388.dp7 = ((double) bmp388.p7 / temp_var);

		bmp388.p8 = (int8_t) bmp388.buffer[17];
		temp_var = 32768.0f; //2^15
		bmp388.dp8 = ((double) bmp388.p8 / temp_var);

		bmp388.p9 = (int16_t) BMP3_CONCAT_BYTES(bmp388.buffer[19], bmp388.buffer[18]);
		temp_var = 281474976710656.0f; //2^48
		bmp388.dp9 = ((double) bmp388.p9 / temp_var);

		bmp388.p10 = (int8_t) bmp388.buffer[20];
		bmp388.dp10 = ((double) bmp388.p10 / temp_var);

		bmp388.p11 = (int8_t) bmp388.buffer[21];
		temp_var = 36893488147419103232.0f; //2^65
		bmp388.dp11 = ((double) bmp388.p11 / temp_var);
	}
	return status;
}

uint8_t bmp388SoftReset() {
	bmp388.buffer[1] = BMP3_PWR_SOFT_RESET;
	uint8_t status = spi3Write(BMP3_CMD_ADDR, bmp388.buffer, 1, BP388_SPI3_DEVICE);
	delayMs(10);
	return status;
}

uint8_t bmp388SetOSR() {
	uint8_t status = 0;
	if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_LOW_POWER) {
		bmp388.buffer[0] = ((uint8_t) BMP3_NO_OVERSAMPLING_1X << 3) | (uint8_t) BMP3_OVERSAMPLING_2X;  //Temp OSR | Pres OSR
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_STD) {
		bmp388.buffer[0] = ((uint8_t) BMP3_NO_OVERSAMPLING_1X << 3) | (uint8_t) BMP3_OVERSAMPLING_4X;  //Temp OSR | Pres OSR
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_HIGH) {
		bmp388.buffer[0] = ((uint8_t) BMP3_NO_OVERSAMPLING_1X << 3) | (uint8_t) BMP3_OVERSAMPLING_8X;    //Temp OSR | Pres OSR
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_ULTRA_HIGH) {
		bmp388.buffer[0] = ((uint8_t) BMP3_OVERSAMPLING_2X << 3) | (uint8_t) BMP3_OVERSAMPLING_16X; //Temp OSR | Pres OSR
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_HIGHEST) {
		bmp388.buffer[0] = ((uint8_t) BMP3_OVERSAMPLING_4X << 3) | (uint8_t) BMP3_OVERSAMPLING_32X; //Temp OSR | Pres OSR
	}
	if (spi3Write(BMP3_OSR_ADDR, bmp388.buffer, 1, BP388_SPI3_DEVICE)) {
		delayMs(10);
		status = 1;
	}
	return status;
}

uint8_t bmp388SetIIRFilter() {
	uint8_t status = 0;
	if (BMP388_ENABLE_LPF == 0 || BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_LOW_POWER) {
		bmp388.buffer[0] = BMP3_IIR_FILTER_DISABLE;
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_STD) {
		bmp388.buffer[0] = BMP3_IIR_FILTER_COEFF_1;
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_HIGH) {
		bmp388.buffer[0] = BMP3_IIR_FILTER_COEFF_3;
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_ULTRA_HIGH) {
		bmp388.buffer[0] = BMP3_IIR_FILTER_COEFF_7;
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_HIGHEST) {
		bmp388.buffer[0] = BMP3_IIR_FILTER_COEFF_15;
	}
	if (spi3Write(BMP3_CONFIG_ADDR, bmp388.buffer, 1, BP388_SPI3_DEVICE)) {
		delayMs(10);
		status = 1;
	}
	return status;
}

uint8_t bmp388SetODR() {
	uint8_t status = 0;
	if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_LOW_POWER) {
		bmp388.buffer[0] = BMP3_ODR_200_HZ;
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_STD) {
		bmp388.buffer[0] = BMP3_ODR_100_HZ;
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_HIGH) {
		bmp388.buffer[0] = BMP3_ODR_50_HZ;
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_ULTRA_HIGH) {
		bmp388.buffer[0] = BMP3_ODR_25_HZ;
	} else if (BMP388_RESOLUTION_TYPE == BMP388_RESOLUTION_TYPE_HIGHEST) {
		bmp388.buffer[0] = BMP3_ODR_12_5_HZ;
	}
	if (spi3Write(BMP3_ODR_ADDR, bmp388.buffer, 1, BP388_SPI3_DEVICE)) {
		delayMs(10);
		status = 1;
	}
	return status;
}

uint8_t bmp388SetPWRControl() {
	uint8_t status = 0;
	bmp388.buffer[0] = (uint8_t) BMP3_NORMAL_MODE | (uint8_t) BMP3_PRESSURE_ENABLE | (uint8_t) BMP3_TEMPERATURE_ENABLE;
	if (spi3Write(BMP3_PWR_CTRL_ADDR, bmp388.buffer, 1, BP388_SPI3_DEVICE)) {
		delayMs(10);
		status = 1;
	}
	return status;
}

uint8_t bmp388DoSettings() {
	uint8_t status = 0;
	if (bmp388SetPWRControl()) {
		if (bmp388SetODR()) {
			if (bmp388SetIIRFilter()) {
				if (bmp388SetOSR()) {
					status = 1;
				}
			}
		}
	}
	return status;
}

uint8_t bmp388Init() {
	uint8_t status = initSPI3();
	if (status) {
		logString("[bmp388,IO:SPI-3] > Success\n");
	} else {
		logString("[bmp388,IO:SPI-3] > Failed\n");
		return 0;
	}
	status = bmp388CheckConnection();
	if (status) {
		logString("[bmp388,CON] > Success\n");
	} else {
		logString("[bmp388,CON] > Failed\n");
		return 0;
	}
	if (bmp388SoftReset()) {
		logString("[bmp388,Reset] > Success\n");
		if (bmpReadCalib()) {
			logString("[bmp388, Calib Read] > Success\n");
			if (bmp388DoSettings()) {
				logString("[bmp388, Settings] > Success\n");
				status = 1;
			} else {
				logString("[bmp388, Settings] > Failed\n");
				return 0;
			}
		} else {
			logString("[bmp388, Calib Read] > Failed\n");
			return 0;
		}
	} else {
		logString("[bmp388, Reset] > Failed\n");
		return 0;
	}
	return status;
}

void bmp388CalculateTemp() {
	float tempData1 = (float) (bmp388.uTemperature - bmp388.dt1);
	float tempData2 = (float) (tempData1 * bmp388.dt2);
	bmp388.temperature = tempData2 + (tempData1 * tempData1) * bmp388.dt3;
}

void bmp388CalculatePressure() {
	double partial_data1 = bmp388.dp6 * bmp388.temperature;

	double tP2 = (double) bmp388.temperature * (double) bmp388.temperature;
	double tP3 = (double) bmp388.temperature * tP2;

	double partial_data2 = bmp388.dp7 * tP2;
	double partial_data3 = bmp388.dp8 * tP3;
	double partial_out1 = bmp388.dp5 + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = bmp388.dp2 * bmp388.temperature;
	partial_data2 = bmp388.dp3 * tP2;
	partial_data3 = bmp388.dp4 * tP3;

	double partial_out2 = bmp388.uPressure * (bmp388.dp1 + partial_data1 + partial_data2 + partial_data3);

	double uP2 = (double) bmp388.uPressure * (double) bmp388.uPressure;
	double uP3 = (double) bmp388.uPressure * uP2;

	partial_data1 = uP2;
	partial_data2 = bmp388.dp9 + bmp388.dp10 * bmp388.temperature;
	partial_data3 = partial_data1 * partial_data2;

	double partial_data4 = partial_data3 + uP3 * bmp388.dp11;
	bmp388.pressure = partial_out1 + partial_out2 + partial_data4;

	bmp388.pressure = bmp388.pressure * BMP388_PRESSURE_OUTPUT_SCALE;
}

void bmp388CalculateAltitude() {
	//bmp388.altitude = (float) ((double)1.0f - pow((double)bmp388.pressure / (double)BMP388_SEALEVEL_PRESSURE, PRESSURE_PWR_CONST)) * PRESSURE_GAS_CONST;
	bmp388.altitude = (1.0f - powf(bmp388.pressure / BMP388_SEALEVEL_PRESSURE, BMP388_PRESSURE_PWR_CONST)) * BMP388_PRESSURE_GAS_CONST;
}

void bmp388ReadData() {
	if (spi3Read(BMP3_DATA_ADDR, bmp388.buffer, 7, BP388_SPI3_DEVICE) == 1) {
		bmp388.uPressure = ((uint32_t)(bmp388.buffer[3] << 16)) | (uint32_t)(bmp388.buffer[2] << 8) | (uint32_t) bmp388.buffer[1];
		bmp388.uTemperature = ((uint32_t)(bmp388.buffer[6] << 16)) | (uint32_t)(bmp388.buffer[5] << 8) | (uint32_t) bmp388.buffer[4];
		bmp388CalculateTemp();
		bmp388CalculatePressure();
		bmp388CalculateAltitude();
	}
}

void bmp388Reset() {
	bmp388SoftReset();
}

