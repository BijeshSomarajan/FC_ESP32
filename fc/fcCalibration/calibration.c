#include "calibration.h"
#include "flash.h"

#define CALIB_STATUS_ADDR CALIB_PROP_MAX_CONFIGURABLE_LENGTH
#define CALIB_STATUS_MAJIC_NUMBER 10

float CONFIG_PROPERTY_VALUE_SCALE = 1000.0f;
float CONFIG_PROPERTY_VALUE_EXTRA_SCALE = 10000.0f;

int32_t CALIB_DATA[CALIB_PROP_LENGTH];

uint8_t initCalibration() {
	return initFlash();
}

int32_t getCalibrationScalableValue(float value) {
	return value * CONFIG_PROPERTY_VALUE_SCALE;
}

int32_t* getCalibrationData() {
	return CALIB_DATA;
}

int32_t getCalibrationValue(uint8_t index) {
	return CALIB_DATA[index];
}

void setCalibrationValue(uint8_t index, int32_t value) {
	CALIB_DATA[index] = value;
}

float getScaledCalibrationValue(uint8_t index) {
	return ((float) CALIB_DATA[index]) / CONFIG_PROPERTY_VALUE_SCALE;
}

void setScaledCalibrationValue(uint8_t index, float value) {
	CALIB_DATA[index] = (int32_t)(value * CONFIG_PROPERTY_VALUE_SCALE);
}

void setExtraScaledCalibrationValue(uint8_t index, float value) {
	CALIB_DATA[index] = (int32_t)(value * CONFIG_PROPERTY_VALUE_EXTRA_SCALE);
}

/************************************************************************/
/* Sets the default calibration values                                  */
/************************************************************************/
void setDefaultCalibration() {
	//PID Master Calibrations
	CALIB_DATA[CALIB_PROP_PID_KP_PITCH_ADDR] = 980;
	CALIB_DATA[CALIB_PROP_PID_KP_ROLL_ADDR] = 980;
	CALIB_DATA[CALIB_PROP_PID_KP_YAW_ADDR] = 980;

	CALIB_DATA[CALIB_PROP_PID_KI_PITCH_ADDR] = 0;
	CALIB_DATA[CALIB_PROP_PID_KI_ROLL_ADDR] = 0;
	CALIB_DATA[CALIB_PROP_PID_KI_YAW_ADDR] = 0;

	CALIB_DATA[CALIB_PROP_PID_PITCH_LIMIT_ADDR] = 150;
	CALIB_DATA[CALIB_PROP_PID_ROLL_LIMIT_ADDR] = 150;
	CALIB_DATA[CALIB_PROP_PID_YAW_LIMIT_ADDR] = 150;

	//PID Inner Calibrations
	CALIB_DATA[CALIB_PROP_RATE_PID_KP_PITCH_ADDR] = 950;
	CALIB_DATA[CALIB_PROP_RATE_PID_KP_ROLL_ADDR] = 950;
	CALIB_DATA[CALIB_PROP_RATE_PID_KP_YAW_ADDR] = 950;

	CALIB_DATA[CALIB_PROP_RATE_PID_KD_PITCH_ADDR] = 0;
	CALIB_DATA[CALIB_PROP_RATE_PID_KD_ROLL_ADDR] = 0;
	CALIB_DATA[CALIB_PROP_RATE_PID_KD_YAW_ADDR] = 0;

	CALIB_DATA[CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR] = 50;
	CALIB_DATA[CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR] = 50;
	CALIB_DATA[CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR] = 50;

	//Inner PID limit
	CALIB_DATA[CALIB_PROP_PID_PITCH_LIMIT_ADDR] = 150;
	CALIB_DATA[CALIB_PROP_PID_ROLL_LIMIT_ADDR] = 150;
	CALIB_DATA[CALIB_PROP_PID_YAW_LIMIT_ADDR] = 250;

	//Default Acc Bias configurations
	CALIB_DATA[ CALIB_PROP_AX_BIAS_ADDR] = 482;
	CALIB_DATA[ CALIB_PROP_AY_BIAS_ADDR] = 117;
	CALIB_DATA[ CALIB_PROP_AZ_BIAS_ADDR] = 8706;

	//Default Gyro Bias configurations
	CALIB_DATA[ CALIB_PROP_GX_BIAS_ADDR] = -96;
	CALIB_DATA[ CALIB_PROP_GY_BIAS_ADDR] = -41;
	CALIB_DATA[ CALIB_PROP_GZ_BIAS_ADDR] = -454;

	//Default mag Bias configurations
	CALIB_DATA[ CALIB_PROP_MX_OFFSET_ADDR] = 1195;
	CALIB_DATA[ CALIB_PROP_MY_OFFSET_ADDR] = 1199;
	CALIB_DATA[ CALIB_PROP_MZ_OFFSET_ADDR] = 1148;

	//Default mag Bias configurations
	CALIB_DATA[ CALIB_PROP_MX_BIAS_ADDR] = 91404;
	CALIB_DATA[ CALIB_PROP_MY_BIAS_ADDR] = 571794;
	CALIB_DATA[ CALIB_PROP_MZ_BIAS_ADDR] = 154636;

	//Default mag Bias configurations
	CALIB_DATA[ CALIB_PROP_MX_SCALE_ADDR] = 1030;
	CALIB_DATA[ CALIB_PROP_MY_SCALE_ADDR] = 1011;
	CALIB_DATA[ CALIB_PROP_MZ_SCALE_ADDR] = 960;

	//IMU Temperature configurations
	CALIB_DATA[CALIB_PROP_IMU_TEMP_ADDR] = 11111;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AX_C_ADDR] = -1;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AX_C1_ADDR] = -117;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AX_C2_ADDR] = 49;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AX_C3_ADDR] = -22;

	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AY_C_ADDR] = 3;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AY_C1_ADDR] = 28;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AY_C2_ADDR] = -32;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AY_C3_ADDR] = 15;

	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AZ_C_ADDR] = -7;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AZ_C1_ADDR] = 672;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AZ_C2_ADDR] = -695;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_AZ_C3_ADDR] = 255;

	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GX_C_ADDR] = -271;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GX_C1_ADDR] = 1139;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GX_C2_ADDR] = -3716;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GX_C3_ADDR] = 2211;

	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GY_C_ADDR] = -70;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GY_C1_ADDR] = -4705;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GY_C2_ADDR] = 2492;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GY_C3_ADDR] = -1141;

	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GZ_C_ADDR] = -202;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GZ_C1_ADDR] = -65335;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GZ_C2_ADDR] = -1252;
	CALIB_DATA[CALIB_PROP_IMU_TEMP_COEFF_GZ_C3_ADDR] = 884;

	//Stick Rates
	CALIB_DATA[CALIB_PROP_RC_THROTTLE_RATE_K_ADDR] = 1000;
	CALIB_DATA[CALIB_PROP_RC_PITCH_RATE_P_ADDR] = 200;
	CALIB_DATA[CALIB_PROP_RC_PITCH_RATE_I_ADDR] = 0;
	CALIB_DATA[CALIB_PROP_RC_ROLL_RATE_P_ADDR] = 200;
	CALIB_DATA[ CALIB_PROP_RC_ROLL_RATE_I_ADDR] = 0;
	CALIB_DATA[CALIB_PROP_RC_YAW_RATE_P_ADDR] = 400;
	CALIB_DATA[CALIB_PROP_RC_YAW_RATE_I_ADDR] = 0;

	//Stick Offsets
	CALIB_DATA[CALIB_PROP_RC_THROTTLE_OFFSET_ADDR] = 1000;
	CALIB_DATA[CALIB_PROP_RC_PITCH_OFFSET_ADDR] = 1511;
	CALIB_DATA[CALIB_PROP_RC_ROLL_OFFSET_ADDR] = 1509;
	CALIB_DATA[CALIB_PROP_RC_YAW_OFFSET_ADDR] = 1506;
	CALIB_DATA[CALIB_PROP_RC_LIFTOFF_THROTTLE_ADDR] = 250;

	//PID configuration for altitude hold
	CALIB_DATA[CALIB_PROP_ALT_HOLD_PID_KP_ADDR] = 3200;
	CALIB_DATA[CALIB_PROP_ALT_HOLD_PID_KD_ADDR] = 0;
	CALIB_DATA[CALIB_PROP_ALT_HOLD_RATE_PID_KP_ADDR] = 600;
	CALIB_DATA[CALIB_PROP_ALT_HOLD_RATE_PID_KD_ADDR] = -400;
	CALIB_DATA[CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR] = 300;
	CALIB_DATA[CALIB_PROP_ALT_HOLD_MAX_TERRAIN_HEIGHT_ADDR] = 120; //12 mts
	CALIB_DATA[CALIB_PROP_ALT_HOLD_MAX_ASL_HEIGHT_ADDR] = 500; //50 mts

	//PID configuration for POS hold
	CALIB_DATA[CALIB_PROP_POS_HOLD_PID_KP_ADDR] = 600;
	CALIB_DATA[CALIB_PROP_POS_HOLD_PID_KD_ADDR] = 0;
	CALIB_DATA[CALIB_PROP_POS_HOLD_RATE_PID_KP_ADDR] = 90;
	CALIB_DATA[CALIB_PROP_POS_HOLD_RATE_PID_KD_ADDR] = -3;
	CALIB_DATA[CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR] = 30;

	CALIB_DATA[CALIB_PROP_FLIGHT_MODEL_ADDR] = 4;
}

/******************************************************************************/
/* Persists the data to the persistence store , updates the calibration status*/
/******************************************************************************/
uint8_t saveCalibration() {
	CALIB_DATA[CALIB_STATUS_ADDR] = CALIB_STATUS_MAJIC_NUMBER;
	return writeWordsToFlash(CALIB_DATA, CALIB_PROP_LENGTH);
}

/************************************************************************/
/* Loads the calibration data from EEProm                               */
/************************************************************************/
uint8_t loadCalibration() {
	uint8_t status = readWordsFromFlash(CALIB_DATA, CALIB_PROP_LENGTH);
	return status;
}

/************************************************************************/
/* Checks if the calibration was done                                   */
/************************************************************************/
uint8_t isCalibrated() {
	if (readWordFromFlash(CALIB_STATUS_ADDR) == CALIB_STATUS_MAJIC_NUMBER) {
		return 1;
	} else {
		return 0;
	}
}

