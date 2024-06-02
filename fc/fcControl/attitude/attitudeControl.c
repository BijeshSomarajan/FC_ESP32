#include "attitudeControl.h"
#include "lowPassFilter.h"
#include "calibration.h"
#include "pid.h"
#include "attitudeSensor.h"
#include "control.h"
#include "imu.h"
#include "fcStatus.h"

void configureAttitudeControl(float rollPitchRatio);
//Attitude PID references
PID attitudePitchPID, attitudeRollPID, attitudeYawPID, attitudePitchRatePID, attitudeRollRatePID, attitudeYawRatePID;

/**
 * Initializes the attitude control
 */
uint8_t initAttitudeControl() {
	/* Attitude PID initiation*/
	pidInit(&attitudePitchPID, getScaledCalibrationValue(CALIB_PROP_PID_KP_PITCH_ADDR), getScaledCalibrationValue(CALIB_PROP_PID_KI_PITCH_ADDR), 0, 0);
	pidInit(&attitudeRollPID, getScaledCalibrationValue(CALIB_PROP_PID_KP_ROLL_ADDR), getScaledCalibrationValue(CALIB_PROP_PID_KI_ROLL_ADDR), 0, 0);
	pidInit(&attitudeYawPID, getScaledCalibrationValue(CALIB_PROP_PID_KP_YAW_ADDR), getScaledCalibrationValue(CALIB_PROP_PID_KI_YAW_ADDR), 0, 0);
	//Set overall limit
	pidSetPIDOutputLimits(&attitudePitchPID, -getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR));
	pidSetPIDOutputLimits(&attitudeRollPID, -getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR));
	pidSetPIDOutputLimits(&attitudeYawPID, -getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR));
	//Set P limit
	pidSetPOutputLimits(&attitudePitchPID, -getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR));
	pidSetPOutputLimits(&attitudeRollPID, -getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR));
	pidSetPOutputLimits(&attitudeYawPID, -getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR));

	//Set I limit
	pidSetIOutputLimits(&attitudePitchPID, -getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR));
	pidSetIOutputLimits(&attitudeRollPID, -getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR));
	pidSetIOutputLimits(&attitudeYawPID, -getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR));

	/* Attitude Rate PID Settings*/
	pidInit(&attitudePitchRatePID, getScaledCalibrationValue(CALIB_PROP_RATE_PID_KP_PITCH_ADDR), 0, getScaledCalibrationValue(CALIB_PROP_RATE_PID_KD_PITCH_ADDR), ATT_CONTROL_D_RATE_LPF_FREQ);
	pidInit(&attitudeRollRatePID, getScaledCalibrationValue(CALIB_PROP_RATE_PID_KP_ROLL_ADDR), 0, getScaledCalibrationValue(CALIB_PROP_RATE_PID_KD_ROLL_ADDR), ATT_CONTROL_D_RATE_LPF_FREQ);
	pidInit(&attitudeYawRatePID, getScaledCalibrationValue(CALIB_PROP_RATE_PID_KP_YAW_ADDR), 0, getScaledCalibrationValue(CALIB_PROP_RATE_PID_KD_YAW_ADDR), ATT_CONTROL_D_RATE_LPF_FREQ);

	//Set overall limit
	pidSetPIDOutputLimits(&attitudePitchRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR));
	pidSetPIDOutputLimits(&attitudeRollRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR));
	pidSetPIDOutputLimits(&attitudeYawRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR));
	//Set P limit
	pidSetPOutputLimits(&attitudePitchRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR));
	pidSetPOutputLimits(&attitudeRollRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR));
	pidSetPOutputLimits(&attitudeYawRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR));

	//Set D limit
	pidSetDOutputLimits(&attitudePitchRatePID, -getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR));
	pidSetDOutputLimits(&attitudeRollRatePID, -getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR));
	pidSetDOutputLimits(&attitudeYawRatePID, -getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR));
	return 1;
}

/**
 * Resets Attitude control
 */
void resetAttitudeControl(uint8_t hard) {
	if (hard) {
		//Reset the master PIDs
		pidReset(&attitudePitchPID);
		pidReset(&attitudeRollPID);
		pidReset(&attitudeYawPID);
		//Reset the rate PIDs
		pidReset(&attitudePitchRatePID);
		pidReset(&attitudeRollRatePID);
		pidReset(&attitudeYawRatePID);
		//Reset process control values
		controlData.pitchControl = 0.0f;
		controlData.rollControl = 0.0f;
		controlData.yawControl = 0.0f;
	}
	//Reset the master PIDs
	pidResetI(&attitudePitchPID);
	pidResetI(&attitudeRollPID);
	pidResetI(&attitudeYawPID);
	//Reset the master PIDs
	pidResetI(&attitudePitchRatePID);
	pidResetI(&attitudeRollRatePID);
	pidResetI(&attitudeYawRatePID);
}

/************************************************************************/
/**
 * Gets the yaw difference considering the switch from 0 - 360
 * Assuming there is no Yaw movement more than 180 deg
 */
float updateHeadingDelta() {
	float headingDiff = imuData.heading - fcStatusData.headingRef;
	//Crossing over 0 to left
	if (headingDiff >= 180.0f) {
		headingDiff += -360.0f;
	} else if (headingDiff <= -180.0f) {
		//Crossing over 0 to right
		headingDiff += 360.0f;
	}
	return headingDiff;
}

void controlAttitude(float dt, float expectedPitch, float expectedRoll, float expectedYaw, float pitchRollPGain, float pitchRollIGain, float pitchRollRatePGain, float pitchRollRateDGain) {
	float headingDelta = updateHeadingDelta();
	//Apply attitude PIDs
	pidUpdateWithGains(&attitudePitchPID, -imuData.pitch, expectedPitch, dt, pitchRollPGain, pitchRollIGain, 0);
	pidUpdateWithGains(&attitudeRollPID, -imuData.roll, expectedRoll, dt, pitchRollPGain, pitchRollIGain, 0);
	pidUpdate(&attitudeYawPID, -headingDelta, -expectedYaw, dt);

	//Apply attitude rate PIDs
	pidUpdateWithGains(&attitudePitchRatePID, -imuData.pitchRate, -attitudePitchPID.pid, dt, pitchRollRatePGain, 0, pitchRollRateDGain);
	pidUpdateWithGains(&attitudeRollRatePID, -imuData.rollRate, -attitudeRollPID.pid, dt, pitchRollRatePGain, 0, pitchRollRateDGain);
	pidUpdate(&attitudeYawRatePID, -imuData.yawRate, -attitudeYawPID.pid, dt);

	//Copy the values for output
	controlData.pitchControl = attitudePitchRatePID.pid;
	controlData.rollControl = attitudeRollRatePID.pid;
	controlData.yawControl = attitudeYawRatePID.pid;
}

