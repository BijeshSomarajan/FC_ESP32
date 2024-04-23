#include "indicatorSensor.h"
#include "indicator.h"

uint8_t initIndicatorSensors() {
	return initIndicators();
}

void statusIndicatorSensorOn() {
	statusIndicatorOn();
}

void statusIndicatorSensorOff() {
	statusIndicatorOff();
}

void statusIndicatorSensorToggle() {
	statusIndicatorToggle();
}

