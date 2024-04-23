#ifndef ALTITUDESENSOR_H_
#define ALTITUDESENSOR_H_

#include <stdio.h>
#include <inttypes.h>
#include "bmp388.h"
#include "fcLogger.h"

typedef struct _ALTITUDE_DATA ALTITUDE_DATA;
struct _ALTITUDE_DATA {
	float altitudeSeaLevelHome;
	float altitudeSeaLevel;
	float altitudeSeaLevelRaw;
	float verticalVelocity;
	float dt;
	uint8_t cpu;
};
extern ALTITUDE_DATA altitudeData;

#define SENSOR_ALT_SAMPLE_PERIOD  BMP388_SAMPLE_PRRIOD

uint8_t initAltitudeSensors(void);
void resetAltitudeSensors(void);
void readAltitudeSensors(float dt);
void stabilizeAltitudeSensors();

#endif
