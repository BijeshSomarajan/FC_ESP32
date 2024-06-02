#include "rcSensor.h"

#if WIFI_RC_SENSOR_ENABLED != 1

#include "calibration.h"
#include "delayTimer.h"
#include "fcLogger.h"
#include "indicatorSensor.h"
#include "fsia.h"

#define RC_CALIBRATION_COUNT  100

uint8_t initRCSensors() {
	uint8_t status = initFSIA();
	if (status) {
		logString("[rc] > Success\n");
	} else {
		logString("[rc] > Failed\n");
	}
	return status;
}

void readRCSensors() {
	readFSIA();
}

uint16_t getRCValue(uint8_t channel) {
	return getFSIAChannelValue(channel);
}

void resetRcSensors() {

}

void calibrateRCSensor() {
	uint32_t thTotal = 0, pitchTotal = 0, rollTotal = 0, yawTotal = 0;
	for (uint16_t indx = 0; indx < RC_CALIBRATION_COUNT; indx++) {
		thTotal += getRCValue(RC_TH_CHANNEL_INDEX);
		pitchTotal += getRCValue(RC_PITCH_CHANNEL_INDEX);
		rollTotal += getRCValue(RC_ROLL_CHANNEL_INDEX);
		yawTotal += getRCValue(RC_YAW_CHANNEL_INDEX);
		delayMs(50);
		statusIndicatorSensorToggle();
	}
	setCalibrationValue(CALIB_PROP_RC_THROTTLE_OFFSET_ADDR, (uint16_t) (thTotal / RC_CALIBRATION_COUNT));
	setCalibrationValue(CALIB_PROP_RC_PITCH_OFFSET_ADDR, (uint16_t) (pitchTotal / RC_CALIBRATION_COUNT));
	setCalibrationValue(CALIB_PROP_RC_ROLL_OFFSET_ADDR, (uint16_t) (rollTotal / RC_CALIBRATION_COUNT));
	setCalibrationValue(CALIB_PROP_RC_YAW_OFFSET_ADDR, (uint16_t) (yawTotal / RC_CALIBRATION_COUNT));
	saveCalibration();
	statusIndicatorSensorOff();
}

#endif
