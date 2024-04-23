#include "rcManager.h"
#include "rcSensor.h"
#include "fcStatus.h"
#include "calibration.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "mathUtil.h"
#include "fcLogger.h"
#include "delayTimer.h"
#include "deltaTimer.h"
#include "indicatorSensor.h"
#include "managerConfig.h"

TaskHandle_t rcMgrTaskHandle;

#define RC_FAIL_SAFE_ACTIVATION_PERIOD  1.0f // Fail safe activated after a second

//Stick dead bands
#define THROTTLE_CENTER_DEADBAND  10
#define YAW_CENTER_DEADBAND  10
#define PITCH_CENTER_DEADBAND  10
#define ROLL_CENTER_DEADBAND  10

#define INIT_RC_MID_SAMPLE_COUNT 10

//Internal functions
int16_t applyStickDeadBand(int16_t rcChannelValue);
void loadRCStickMids(void);
void loadRCStickDelta(void);
uint8_t canStartModel(void);
uint8_t canArmModel(void);
uint8_t isThrottleCentered(float dt);
uint8_t isYawCentered(float dt);
uint8_t isRollCentered(float dt);
uint8_t isPitchCentered(float dt);
void applyRCStickEffectiveness();

uint8_t canEnableAltHold(void);
void configureRCStickControl(void);
uint8_t canEnableGlobalPosHold(void);
uint8_t canEnableTerrainPosHold(void);
uint8_t canEnableRTH(void);
uint8_t isGlobalPosHoldModeActive(void);
uint8_t isRTHModeActive(void);
uint8_t isHeadLessModeActive(void);
uint8_t canEnableLandingMode();
void checkForFailSafe(float dt);

//Inner state variables
float rcDataReadDt = 0;
float rcDataProcessDt = 0;
float rcFailsafeDt = 0;

float rcStickPitchGain = 0;
float rcStickRollGain = 0;
float rcStickYawGain = 0;
float rcStickThrottleGain = 0;

/**
 * Stick threshold for activating position hold
 */
int16_t posHoldModeActivationTsh = RC_CHANNEL_MID_VALUE * 0.9f;

/**
 * Stick threshold for activating RTH
 */
int16_t rthModeActivationTsh = RC_CHANNEL_MAX_VALUE * 0.9f;

/**
 * Stick threshold for landing
 */
int16_t landingModeActivationTsh = RC_CHANNEL_MID_VALUE * 0.9f;

/**
 * Stick threshold for landing
 */
int16_t headLessModeActivationTsh = RC_CHANNEL_MID_VALUE * 0.9f;

/**
 * Initializes the Radio Controller
 */
uint8_t initRCManager() {
	uint8_t status = initRCSensors();
	if (!status) {
		logString("init[sensor,rc] > Failure\n");
	}
	logString("init[sensor,rc] > Success\n");
	logString("RC : Data handler -> Initialized\n");
	loadRCStickMids();
	configureRCStickControl();
	logString("RC : Stick controls -> Initialized\n");
	return 1;
}

/**
 * Process the RC Data
 */
void processRCData(float rcUpdateDt) {
	//Loads the stick delta
	loadRCStickDelta();

	rcData.RC_DELTA_DATA[RC_FLIGHT_MODE_CHANNEL_INDEX] = getRCValue(RC_FLIGHT_MODE_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_AUX_CHANNEL5_INDEX] = getRCValue(RC_AUX_CHANNEL5_INDEX);

	fcStatusData.canStart = canStartModel();
	fcStatusData.canArm = (fcStatusData.canStart ? canArmModel() : 0);
	//Capture stick centers
	rcData.pitchCentered = isPitchCentered(rcUpdateDt);
	rcData.rollCentered = isRollCentered(rcUpdateDt);
	rcData.yawCentered = isYawCentered(rcUpdateDt);
	rcData.throttleCentered = isThrottleCentered(rcUpdateDt);
	//Apply RC stick rates
	applyRCStickEffectiveness();
	//Set the FC status
	fcStatusData.enableAltitudeHold = canEnableAltHold();
	//Note order is important
	fcStatusData.isRTHModeActive = isRTHModeActive();
	fcStatusData.isGlobalPosHoldModeActive = isGlobalPosHoldModeActive();
	fcStatusData.enableGlobalPositionHold = canEnableGlobalPosHold();
	fcStatusData.enableTerrainPosHold = canEnableTerrainPosHold();
	fcStatusData.enableRTH = canEnableRTH();
	//fcStatusData.isTerrainAltModeActive = isTerrainAltModeActive();
	fcStatusData.isLandingModeActive = canEnableLandingMode();
	fcStatusData.isHeadLessModeActive = isHeadLessModeActive();

	//Check For activating fail safe
	checkForFailSafe(rcUpdateDt);
}

#if RC_MGR_USE_TIMER == 1
int64_t rcUpdateLastTimeUs = 0;
void updateRCDataTask(void *pvParameters) {
	rcData.cpu = xPortGetCoreID();
	int64_t currentTimeUs = getTimeUSec();
	float dt = getUSecTimeInSec(currentTimeUs - rcUpdateLastTimeUs);
	rcUpdateLastTimeUs = currentTimeUs;
	rcData.readDt = dt;
	readRCSensors();
}

int64_t rcDataLoadLastTimeUs = 0;
void processRCDataTask(void *pvParameters) {
	int64_t currentTimeUs = getTimeUSec();
	float dt = getUSecTimeInSec(currentTimeUs - rcDataLoadLastTimeUs);
	rcDataLoadLastTimeUs = currentTimeUs;
	rcData.processDt = dt;
	processRCData(dt);
}
const esp_timer_create_args_t rcUpdatePeriodicTimerArgs = { .callback = &updateRCDataTask, .name = "rcDataUpdate" };
esp_timer_handle_t rcDataUpdatePeriodicTimer;

const esp_timer_create_args_t rcDataLoadPeriodicTimerArgs = { .callback = &processRCDataTask, .name = "rcDataProcessTask" };
esp_timer_handle_t rcDataLoadPeriodicTimer;

uint8_t rcManagerCreateTimers() {
	uint8_t status = 0;
	esp_err_t err = esp_timer_create(&rcUpdatePeriodicTimerArgs, &rcDataUpdatePeriodicTimer);
	if (err == ESP_OK) {
		logString("RC Manager , Data Update Timer Created , Success!\n");
		err = esp_timer_create(&rcDataLoadPeriodicTimerArgs, &rcDataLoadPeriodicTimer);
		if (err == ESP_OK) {
			logString("RC Manager , Data Load Timer Created , Success!\n");
			status = 1;
		} else {
			logString("RC Manager , Data Load Timer Created , Failed!\n");
		}
	} else {
		logString("RC Manager , Data Update Timer Created , Failed!\n");
	}
	return status;
}

uint8_t rcManagerStartTimers() {
	uint8_t status = 0;
	esp_err_t err = esp_timer_start_periodic(rcDataUpdatePeriodicTimer, RC_DATA_READ_PERIOD_US);
	if (err == ESP_OK) {
		logString("RC Manager , Start Timers, RC Data Update Timer Start , Success!\n");
		err = esp_timer_start_periodic(rcDataLoadPeriodicTimer, RC_DATA_PROCESS_PERIOD_US);
		if (err == ESP_OK) {
			logString("RC Manager , Start Timers, RC Data Load Timer Start , Success!\n");
			status = 1;
		} else {
			logString("RC Manager , Start Timers, RC Data Load Timer Start , Failed!\n");
		}
	} else {
		logString("RC Manager , Start Timers, RC Data Update Timer Start , Failed!\n");
	}
	return status;
}

void rcManagerTimerTask(void *pvParameters) {
	if (rcManagerCreateTimers()) {
		if (rcManagerStartTimers()) {
			logString("RC Manager , Timer Start success , Task running!\n");
			for (;;) {
				vTaskDelay(1);
			}
		} else {
			logString("RC Manager , Timer Start failed , Task Exiting!\n");
		}
	} else {
		logString("RC Manager , Timer Creation failed , Task Exiting!\n");
	}
}

uint8_t startRCManager() {
	BaseType_t err = xTaskCreatePinnedToCore(rcManagerTimerTask, "rcManager", RC_MANAGER_STACK_SIZE, NULL, RC_MANAGER_PRIORITY, &rcMgrTaskHandle, RC_MANAGER_CPU);
	return (err == pdPASS);
}
#else

void rcManagerTask(void *pvParameters) {
	rcData.cpu = xPortGetCoreID();
	int64_t prevTimeUs = getTimeUSec();
	int64_t currentTimeUs = 0;
	float dt = 0;
	for (;;) {
		currentTimeUs = getTimeUSec();
		dt = getUSecTimeInSec(currentTimeUs - prevTimeUs);
		prevTimeUs = currentTimeUs;
		rcDataReadDt += dt;
		rcDataProcessDt += dt;
		if (rcDataReadDt >= RC_DATA_READ_PERIOD) {
			readRCSensors();
			rcData.readDt = rcDataReadDt;
			rcDataReadDt = 0;
		}
		if (rcDataProcessDt >= RC_DATA_PROCESS_PERIOD) {
			processRCData(rcDataProcessDt);
			rcData.processDt = rcDataProcessDt;
			rcDataProcessDt = 0;
		}
		vTaskDelay(1);
	}
}

uint8_t startRCManager() {
	BaseType_t err = xTaskCreatePinnedToCore(rcManagerTask, "rcManager", RC_MANAGER_STACK_SIZE, NULL, RC_MANAGER_PRIORITY, &rcMgrTaskHandle, RC_MANAGER_CPU);
	return (err == pdPASS);
}
#endif

uint8_t stopRCManager() {
	return 1;
}

/**
 * Reset the RC
 */
void resetRCManager() {
//Reset the timer values
	rcDataReadDt = 0;
	rcFailsafeDt = 0;
//Reset RC flags
	rcData.pitchCentered = 0;
	rcData.rollCentered = 0;
	rcData.yawCentered = 0;
	rcData.yawCentered = 0;
//Reset the stick delta and rated values
	rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX] = 0;
	rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX] = 0;
	rcData.RC_EFFECTIVE_DATA[RC_ROLL_CHANNEL_INDEX] = 0;
	rcData.RC_EFFECTIVE_DATA[RC_YAW_CHANNEL_INDEX] = 0;

	rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] = 0;

	rcData.RC_DELTA_DATA[RC_START_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_POS_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_LAND_CHANNEL_INDEX] = 0;
	rcData.RC_DELTA_DATA[RC_HEADING_CHANNEL_INDEX] = 0;
}

/**
 * Configure the stick rate PIDs
 */
void configureRCStickControl() {
	rcStickThrottleGain = getScaledCalibrationValue(CALIB_PROP_RC_THROTTLE_RATE_K_ADDR);
	rcStickPitchGain = getScaledCalibrationValue(CALIB_PROP_RC_PITCH_RATE_P_ADDR);
	rcStickRollGain = getScaledCalibrationValue(CALIB_PROP_RC_ROLL_RATE_P_ADDR);
	rcStickYawGain = getScaledCalibrationValue(CALIB_PROP_RC_YAW_RATE_P_ADDR);
}

void checkForFailSafe(float dt) {
	rcFailsafeDt += dt;
	if (rcFailsafeDt >= RC_FAIL_SAFE_ACTIVATION_PERIOD) {
		if (!isFSIAActiveState()) {
			fcStatusData.isFailSafeLandingMode = 1;
		}
		resetFSIAState();
		rcFailsafeDt = 0;
	}
}

int16_t getThrottleChannelValue() {
	return rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX];
}

int16_t getPitchChannelValue() {
	return rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX];
}

int16_t getRollChannelValue() {
	return rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX];;
}

int16_t getYawChannelValue() {
	return rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX];;
}

/**
 * Apply the RC stick effectiveness
 */
void applyRCStickEffectiveness() {
	rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] * rcStickThrottleGain;
	rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] * rcStickPitchGain;
	rcData.RC_EFFECTIVE_DATA[RC_ROLL_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] * rcStickRollGain;
	rcData.RC_EFFECTIVE_DATA[RC_YAW_CHANNEL_INDEX] = rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] * rcStickYawGain;
}

/************************************************************************/
/*Loads the stick middle values                                         */
/************************************************************************/
void loadRCStickMids() {
	rcData.RC_MID_DATA[RC_TH_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_THROTTLE_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_PITCH_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_PITCH_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_ROLL_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_ROLL_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_YAW_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_YAW_OFFSET_ADDR);
}

/************************************************************************/
/* Get RC Stick Delta by subtracting from the stick middle values        */
/************************************************************************/
void loadRCStickDelta() {
//Load stick deltas
	rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] =  getRCValue(RC_TH_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_TH_CHANNEL_INDEX];
	rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] =  getRCValue(RC_PITCH_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_PITCH_CHANNEL_INDEX];
	rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] =  getRCValue(RC_ROLL_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_ROLL_CHANNEL_INDEX];
	rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] =  getRCValue(RC_YAW_CHANNEL_INDEX) - rcData.RC_MID_DATA[RC_YAW_CHANNEL_INDEX];
//Aux channels
	rcData.RC_DELTA_DATA[RC_POS_CHANNEL_INDEX] =  getRCValue(RC_POS_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_LAND_CHANNEL_INDEX] =  getRCValue(RC_LAND_CHANNEL_INDEX);
	rcData.RC_DELTA_DATA[RC_HEADING_CHANNEL_INDEX] =  getRCValue(RC_HEADING_CHANNEL_INDEX);
//Apply dead bands
	rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX]);
	rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX]);
	rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX]);
	rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX] = applyStickDeadBand(rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX]);
}

/*************************************************************************/
//Checks if the throttle is stable
/*************************************************************************/
uint8_t isThrottleCentered(float dt) {
	if (applyDeadBandInt16(rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX], THROTTLE_CENTER_DEADBAND) != 0) {
		return 0;
	} else {
		return 1;
	}
}

/*************************************************************************/
//Checks if the yaw stick is stable
/*************************************************************************/
uint8_t isYawCentered(float dt) {
	if (applyDeadBandInt16(rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX], YAW_CENTER_DEADBAND) != 0) {
		return 0;
	} else {
		return 1;
	}
}

/*************************************************************************/
//Checks if the Roll stick is stable
/*************************************************************************/
uint8_t isRollCentered(float dt) {
	if (applyDeadBandInt16(rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX], ROLL_CENTER_DEADBAND) != 0) {
		return 0;
	} else {
		return 1;
	}
}

/*************************************************************************/
//Checks if the Pitch stick is stable
/*************************************************************************/
uint8_t isPitchCentered(float dt) {
	if (applyDeadBandInt16(rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX], PITCH_CENTER_DEADBAND) != 0) {
		return 0;
	} else {
		return 1;
	}
}

/**
 * Checks if Position Hold mode is active
 */
uint8_t isHeadLessModeActive() {
	return (rcData.RC_DELTA_DATA[RC_HEADING_CHANNEL_INDEX] > headLessModeActivationTsh);
}

/**
 * Checks if Altitude hold can be enabled
 **/
uint8_t canEnableAltHold() {
//If throttle stick is stable
	if (rcData.throttleCentered) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * Checks if Position Hold mode is active
 */
uint8_t isGlobalPosHoldModeActive() {
	return (rcData.RC_DELTA_DATA[RC_POS_CHANNEL_INDEX] > posHoldModeActivationTsh);
}

/**
 * Checks if Position hold can be enabled
 */
uint8_t canEnableGlobalPosHold() {
//&& rcData.throttleStable
//if (fcStatusData.isGlobalPosHoldModeActive && rcData.pitchCentered && rcData.rollCentered && rcData.yawCentered) {
	if (fcStatusData.isGlobalPosHoldModeActive && rcData.pitchCentered && rcData.rollCentered) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * Checks if Position hold can be enabled
 */
uint8_t canEnableTerrainPosHold() {
//&& rcData.throttleStable
	if (!fcStatusData.isGlobalPosHoldModeActive && rcData.pitchCentered && rcData.rollCentered && rcData.yawCentered) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * Checks if RTH Mode is active
 */
uint8_t isRTHModeActive() {
	return (rcData.RC_DELTA_DATA[RC_POS_CHANNEL_INDEX] > rthModeActivationTsh);
}

/**
 * Checks if Landing Mode is active
 */
uint8_t canEnableLandingMode() {
	return (rcData.RC_DELTA_DATA[RC_LAND_CHANNEL_INDEX] > landingModeActivationTsh);
}

/**
 * Checks if return to home can be enabled
 */
uint8_t canEnableRTH() {
//&& rcData.throttleStable
	if (fcStatusData.isRTHModeActive && rcData.pitchCentered && rcData.rollCentered && rcData.yawCentered) {
		return 1;
	} else {
		return 0;
	}
}

/**
 Checks if the Model can be armed
 **/
uint8_t canStartModel() {
	if (getRCValue(RC_START_CHANNEL_INDEX) == RC_CHANNEL_MAX_VALUE) {
		return 1;
	} else {
		return 0;
	}
}

/**
 Checks if the Model can be armed, The throttle at the bottom position
 **/
uint8_t canArmModel() {
	if (applyDeadBandInt16(getRCValue(RC_TH_CHANNEL_INDEX) - RC_CHANNEL_MIN_VALUE, THROTTLE_CENTER_DEADBAND) == 0 && getRCValue(RC_START_CHANNEL_INDEX) == RC_CHANNEL_MAX_VALUE) {
		return 1;
	} else {
		return 0;
	}
}

/**
 Applies deadband to RC channel value;
 **/
int16_t applyStickDeadBand(int16_t rcChannelValue) {
	if (RC_CHANNEL_DEAD_BAND != 0) {
		return applyDeadBandInt16(rcChannelValue, RC_CHANNEL_DEAD_BAND);
	} else {
		return rcChannelValue;
	}
}

void calibrateRC() {
	calibrateRCSensor();
}
