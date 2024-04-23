#include "altitudeControl.h"
#include "lowPassFilter.h"
#include "calibration.h"
#include "pid.h"
#include "altitudeSensor.h"
#include "control.h"
#include "fcStatus.h"

#define ALT_CONTROL_PID_D_LPF_FREQ   0
#define ALT_CONTROL_RATE_PID_D_LPF_FREQ 0.0f

#define ALT_CONTROL_RATE_D_LIMIT_GAIN 0.8f
#define ALT_CONTROL_RATE_P_LIMIT_GAIN 0.75f

PID altPID;
PID altRatePID;

uint8_t initAltitudeControl() {
	/** Master PID **/
	pidInit(&altPID, getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_PID_KP_ADDR), 0, getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_PID_KD_ADDR), ALT_CONTROL_PID_D_LPF_FREQ);

	pidSetPIDOutputLimits(&altPID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR));
	pidSetPOutputLimits(&altPID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR));
	pidSetDOutputLimits(&altPID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR));

	/** Rate PID **/
	pidInit(&altRatePID, getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_KP_ADDR), 0, getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_KD_ADDR), ALT_CONTROL_RATE_PID_D_LPF_FREQ);

	pidSetPIDOutputLimits(&altRatePID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR));
	pidSetPOutputLimits(&altRatePID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR) * ALT_CONTROL_RATE_P_LIMIT_GAIN, getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR) * ALT_CONTROL_RATE_P_LIMIT_GAIN);
	pidSetDOutputLimits(&altRatePID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR) * ALT_CONTROL_RATE_D_LIMIT_GAIN, getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR) * ALT_CONTROL_RATE_D_LIMIT_GAIN);

	resetAltitudeControl(1);

	return 1;
}

void resetAltitudeControl(uint8_t hard) {
	if (hard) {
		pidReset(&altPID);
		pidReset(&altRatePID);
		controlData.altitudeControl = 0;
	}
	pidResetI(&altPID);
}

void controlAltitudeWithGain(float dt, float altitude, float altitudeRef, float kPGain, float kDGain, float rateKPGain, float rateKDGain) {
	pidUpdateWithGains(&altPID, altitude, altitudeRef, dt, kPGain, 0.0f, kDGain);
	pidUpdateWithGains(&altRatePID, -altitudeData.verticalVelocity, altPID.pid, dt, rateKPGain, 0.0f, rateKDGain);
	controlData.altitudeControl = altRatePID.pid;
}