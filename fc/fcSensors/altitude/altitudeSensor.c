#include "altitudeSensor.h"
#include "lowPassFilter.h"
#include "delayTimer.h"
#include <string.h>

ALTITUDE_DATA altitudeData;

#define SENSOR_ALT_BARO_ALTITUDE_GAIN  10.0f
#define SENSOR_ALT_BARO_LPF_FREQUENCY 0.1f

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
		lowPassFilterInit(&sensorAltBaroLPF, SENSOR_ALT_BARO_LPF_FREQUENCY);
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
	altitudeData.verticalVelocity = 0;
	altitudeData.altitudeSeaLevelHome = 0;
	lowPassFilterReset(&sensorAltBaroLPF);
	bmp388Reset();
}

float getSacledSeaLevelAlt() {
	return bmp388.altitude * SENSOR_ALT_BARO_ALTITUDE_GAIN;
}

/**
 * Reads altitude sensor data
 */
void readAltitudeSensors(float dt) {
	bmp388ReadData();
	altitudeData.altitudeSeaLevelRaw = getSacledSeaLevelAlt();
	altitudeData.altitudeSeaLevel = lowPassFilterUpdate(&sensorAltBaroLPF, altitudeData.altitudeSeaLevelRaw, dt);
}

/**
 * Stabilizes altitude sensor data
 */
void stabilizeAltitudeSensors() {
	bmp388ReadData();
	altitudeData.altitudeSeaLevelRaw = getSacledSeaLevelAlt();
	altitudeData.altitudeSeaLevel = altitudeData.altitudeSeaLevelRaw;
	lowPassFilterResetToValue(&sensorAltBaroLPF, altitudeData.altitudeSeaLevelRaw);
}
