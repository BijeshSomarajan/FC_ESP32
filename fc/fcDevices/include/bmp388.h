#ifndef _BMP388_H_
#define _BMP388_H_
#include <stdio.h>
#include <inttypes.h>

typedef struct _BMP388 BMP388;
struct _BMP388 {
	//Global buffer for read write operations
	uint8_t buffer[24];

	float pressure;
	float temperature;
	float altitude;

	uint32_t uPressure;
	uint32_t uTemperature;

	int32_t dT;
	int64_t off;
	int64_t sens;

	//Configs
	uint16_t t1;
	uint16_t t2;
	int8_t t3;
	int16_t p1;
	int16_t p2;
	int8_t p3;
	int8_t p4;
	uint16_t p5;
	uint16_t p6;
	int8_t p7;
	int8_t p8;
	int16_t p9;
	int8_t p10;
	int8_t p11;

	double dt1;
	double dt2;
	double dt3;
	double dp1;
	double dp2;
	double dp3;
	double dp4;
	double dp5;
	double dp6;
	double dp7;
	double dp8;
	double dp9;
	double dp10;
	double dp11;
};

#define  BMP388_PRESSURE_CALC_SCALE 1.0f
#define  BMP388_PRESSURE_CALC_SCALE 1.0f
#define  BMP388_PRESSURE_OUTPUT_SCALE 0.01f

#define  BMP388_SEALEVEL_PRESSURE  1013.25f * BMP388_PRESSURE_CALC_SCALE
#define  BMP388_PRESSURE_PWR_CONST 0.190295f //0.1902225603956629f
#define  BMP388_PRESSURE_GAS_CONST 44330.0f * BMP388_PRESSURE_CALC_SCALE

#define BMP388_RESOLUTION_TYPE_LOW_POWER 0
#define BMP388_RESOLUTION_TYPE_STD 1
#define BMP388_RESOLUTION_TYPE_HIGH 2
#define BMP388_RESOLUTION_TYPE_ULTRA_HIGH 3
#define BMP388_RESOLUTION_TYPE_HIGHEST 4

#define BMP388_RESOLUTION_TYPE  BMP388_RESOLUTION_TYPE_HIGH
#define BMP388_ENABLE_LPF 0

#define BMP388_SAMPLE_PRRIOD 1.0f/100.0f

uint8_t bmp388CheckConnection(void);
uint8_t bmp388Init(void);
void bmp388ReadData(void);
void bmp388Reset(void);

//Global variable
extern BMP388 bmp388;

#endif
