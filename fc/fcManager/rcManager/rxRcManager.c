#include "rcManager.h"

#if WIFI_RC_MANAGER_ENABLED != 1

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
#include "cpuConfig.h"

TaskHandle_t rcMgrTaskHandle;

#define RC_MGR_USE_TIMER 1
#define RC_DATA_READ_PERIOD  1.0f/2000.0f
#define RC_DATA_READ_PERIOD_US RC_DATA_READ_PERIOD * 1000000.0f

#define RC_DATA_PROCESS_PERIOD  1.0f/100.0f
#define RC_DATA_PROCESS_PERIOD_US RC_DATA_PROCESS_PERIOD * 1000000.0f

#define INIT_RC_MID_SAMPLE_COUNT 10

//Inner state variables
float rcDataReadDt = 0;
float rcDataProcessDt = 0;

/************************************************************************/
/*Loads the stick middle values                                         */
/************************************************************************/
void loadRCStickMids() {
	rcData.RC_MID_DATA[RC_TH_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_THROTTLE_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_PITCH_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_PITCH_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_ROLL_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_ROLL_OFFSET_ADDR);
	rcData.RC_MID_DATA[RC_YAW_CHANNEL_INDEX] = getCalibrationValue(CALIB_PROP_RC_YAW_OFFSET_ADDR);
}

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
	processRCData(dt);
	determineFCState();
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
			vTaskDelete(rcMgrTaskHandle);
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
			determineFCState();
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

void calibrateRC() {
	calibrateRCSensor();
}

#endif
