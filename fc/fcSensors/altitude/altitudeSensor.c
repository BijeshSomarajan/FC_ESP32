#include "altitudeSensor.h"
#include "lowPassFilter.h"
#include "delayTimer.h"
#include <string.h>

ALTITUDE_DATA altitudeData;

#define SENSOR_ALT_BARO_ALTITUDE_GAIN  10.0f

LOWPASSFILTER sensorAltBaroCoarseLPF;
LOWPASSFILTER sensorAltBaroLPF;

float getSacledSeaLevelAlt(void);

/**
 * Initializes altitude sensor
 */
uint8_t initAltitudeSensors() {
	uint8_t status = 0;
	if (bmp388Init()) {
		status = 1;
		logString("Sensor : BMP388 -> Initialized\n");
		lowPassFilterInit(&sensorAltBaroCoarseLPF, SENSOR_ALT_BARO_LPF_FREQUENCY);
		lowPassFilterInit(&sensorAltBaroLPF, SENSOR_ALT_BARO_SMOOTH_LPF_FREQUENCY);
		logString("Sensor : BMP388 -> Filters Initialized\n");
	} else {
		logString("Sensor : BMP388 -> Not Initialized\n");
	}
	return status;
}

/*
 * Resets altitude sensor
 */
void resetAltitudeSensors() {
	altitudeData.altitudeSeaLevelHome = 0;
	lowPassFilterReset(&sensorAltBaroCoarseLPF);
	lowPassFilterReset(&sensorAltBaroLPF);
	bmp388Reset();
}

float getSacledSeaLevelAlt() {
	return bmp388.altitude * SENSOR_ALT_BARO_ALTITUDE_GAIN;
}

void syncToAltCoarseValue(){
	lowPassFilterResetToValue(&sensorAltBaroLPF, altitudeData.altitudeSeaLevelCoarse);
}
/**
 * Reads altitude sensor data
 */
void readAltitudeSensors(float dt) {
	bmp388ReadData();
	altitudeData.altitudeSeaLevelRaw = getSacledSeaLevelAlt();
	altitudeData.altitudeSeaLevelCoarse = lowPassFilterUpdate(&sensorAltBaroCoarseLPF, altitudeData.altitudeSeaLevelRaw, dt);
	altitudeData.altitudeSeaLevel = lowPassFilterUpdate(&sensorAltBaroLPF, altitudeData.altitudeSeaLevelRaw, dt);
}

