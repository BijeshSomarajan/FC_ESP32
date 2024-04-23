#include "altitudeManager.h"
#include "altitudeSensor.h"
#include "fcStatus.h"
#include "calibration.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "mathUtil.h"
#include "fcLogger.h"
#include "deltaTimer.h"
#include "managerConfig.h"
#include "control.h"
#include "lowPassFilter.h"
#include "imu.h"
#include "altitudeControl.h"

TaskHandle_t altitudeMgrTaskHandle;
//Inner state variables
float altitudeUpdateDt = 0;
float altMgrThAggregationDt = 0;
float altMgrAltHoldActivationDt = 0;

LOWPASSFILTER altMgrThrottleControlLPF;
LOWPASSFILTER altMgrVelocityLPF;

void aggregateThrottle(float dt);
void manageAltitude(float dt);

/**
 * Initializes the Radio Controller
 */
uint8_t initAltitudeManager() {
	uint8_t status = initAltitudeSensors();
	if (!status) {
		logString("init[Manager,altitude] > Failure\n");
		return 0;
	}
	lowPassFilterInit(&altMgrThrottleControlLPF, ALT_MGR_THROTTLE_CONTROL_LPF_FREQ);
	lowPassFilterInit(&altMgrVelocityLPF, ALT_MGR_VERTICAL_VELOCITY_LPF_FREQ);
	logString("init[Filters,altitude] > Success\n");
	initAltitudeControl();
	logString("init[Controller,altitude] > Success\n");
	logString("init[Manager,altitude] > Success\n");
	return 1;
}

void stabilizeAltitude() {
	vTaskDelay(2);
	stabilizeAltitudeSensors();
	altitudeData.altitudeSeaLevelHome = altitudeData.altitudeSeaLevel;
	vTaskDelay(2);
}

#if ALTITUDE_MGR_USE_TIMER == 1

int64_t altitudeUpdateLastTimeUs = 0;
void updateAltitudeDataTask(void *pvParameters) {
	altitudeData.cpu = xPortGetCoreID();
	int64_t currentTimeUs = getTimeUSec();
	float dt = getUSecTimeInSec(currentTimeUs - altitudeUpdateLastTimeUs);
	altitudeUpdateLastTimeUs = currentTimeUs;
	altitudeData.dt = dt;
	readAltitudeSensors(dt);
	manageAltitude(dt);
}

const esp_timer_create_args_t altitudeUpdatePeriodicTimerArgs = { .callback = &updateAltitudeDataTask, .name = "updateAltitudeDataTask" };
esp_timer_handle_t altitudeUpdatePeriodicTimer;

uint8_t altitudeManagerCreateTimers() {
	uint8_t status = 0;
	esp_err_t err = esp_timer_create(&altitudeUpdatePeriodicTimerArgs, &altitudeUpdatePeriodicTimer);
	if (err == ESP_OK) {
		logString("Altitude Manager , Data Update Timer Created , Success!\n");
		status = 1;
	} else {
		logString("Altitude Manager , Data Update Timer Created , Failed!\n");
	}
	return status;
}

uint8_t altitudeManagerStartTimers() {
	uint8_t status = 0;
	esp_err_t err = esp_timer_start_periodic(altitudeUpdatePeriodicTimer, SENSOR_ALT_SAMPLE_PERIOD_US);
	if (err == ESP_OK) {
		logString("Altitude Manager , Start Timers,  Data Update Timer Start , Success!\n");
		status = 1;
	} else {
		logString("Altitude Manager , Start Timers, RC Data Update Timer Start , Failed!\n");
	}
	return status;
}

void altitudeManagerTimerTask(void *pvParameters) {
	stabilizeAltitude();
	if (altitudeManagerCreateTimers()) {
		if (altitudeManagerStartTimers()) {
			logString("Altitude Manager , Timer Start success , Task running!\n");
			for (;;) {
				vTaskDelay(1);
			}
		} else {
			logString("Altitude Manager , Timer Start failed , Task Exiting!\n");
		}
	} else {
		logString("Altitude Manager , Timer Creation failed , Task Exiting!\n");
	}
}

uint8_t startAltitudeManager() {
	BaseType_t err = xTaskCreatePinnedToCore(altitudeManagerTimerTask, "altitudeManagerTimerTask", ALTITUDE_MANAGER_STACK_SIZE, NULL, ALTITUDE_MANAGER_PRIORITY, &altitudeMgrTaskHandle, ALTITUDE_MANAGER_CPU);
	return (err == pdPASS);
}
#else

void altitudeManagerTask(void *pvParameters) {
	altitudeData.cpu = xPortGetCoreID();
	int64_t prevTimeUs = getTimeUSec();
	int64_t currentTimeUs = 0;
	float dt = 0;
	stabilizeAltitude();
	logString("Altitude Manager Task, Stabilized Altitude!\n");
	for (;;) {
		currentTimeUs = getTimeUSec();
		dt = getUSecTimeInSec(currentTimeUs - prevTimeUs);
		prevTimeUs = currentTimeUs;
		altitudeUpdateDt += dt;
		if (altitudeUpdateDt >= SENSOR_ALT_SAMPLE_PERIOD) {
			readAltitudeSensors(altitudeUpdateDt);
			manageAltitude(altitudeUpdateDt);
			altitudeData.dt = altitudeUpdateDt;
			altitudeUpdateDt = 0;
		}
		vTaskDelay(1);
	}
}

uint8_t startAltitudeManager() {
	BaseType_t err = xTaskCreatePinnedToCore(altitudeManagerTask, "altitudeManagerTask", ALTITUDE_MANAGER_STACK_SIZE, NULL, ALTITUDE_MANAGER_PRIORITY, &altitudeMgrTaskHandle, ALTITUDE_MANAGER_CPU);
	return (err == pdPASS);
}
#endif

void aggregateThrottle(float dt) {
	altMgrThAggregationDt += dt;
	if (altMgrThAggregationDt >= ALT_MGR_THROTTLE_AGGREGATION_PERIOD) {
		fcStatusData.currentThrottle += rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX] * ALT_MGR_THROTTLE_ALT_AGGREGATION_GAIN;
		fcStatusData.currentThrottle = constrainToRangeF(fcStatusData.currentThrottle, 0, ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA);
		fcStatusData.throttlePercentage = fcStatusData.currentThrottle / ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA;
		altMgrThAggregationDt = 0;
	}
}

void calculateAltVelocity(float dt) {
	float altitudeV = applyDeadBandFloat((imuData.linVz) * ALT_MGR_VERTICAL_VELOCITY_INPUT_GAIN, ALT_MGR_VERTICAL_VELOCITY_DEADBAND);
	altitudeData.verticalVelocity = altitudeV * ALT_MGR_VERTICAL_VELOCITY_OUTPUT_GAIN;
	altitudeData.verticalVelocity = constrainToRangeF(altitudeData.verticalVelocity, -ALT_MGR_VERTICAL_VELOCITY_MAX, ALT_MGR_VERTICAL_VELOCITY_MAX);
	altitudeData.verticalVelocity = lowPassFilterUpdate(&altMgrVelocityLPF, altitudeData.verticalVelocity, dt);
}

void manageAltitude(float dt) {
	calculateAltVelocity(dt);
	if (!rcData.throttleCentered) {
		aggregateThrottle(dt);
		controlData.throttleControl = fcStatusData.currentThrottle;
		fcStatusData.altitudeRefSeaLevel = altitudeData.altitudeSeaLevel;
		resetAltitudeControl(1);
		altMgrAltHoldActivationDt = 0;
		fcStatusData.altitudeHoldEnabled = 0;
	} else {
		if (altMgrAltHoldActivationDt <= ALT_MGR_ALT_HOLD_ACTIVATION_PERIOD) {
			altMgrAltHoldActivationDt += dt;
			fcStatusData.altitudeRefSeaLevel = altitudeData.altitudeSeaLevel;
			fcStatusData.altitudeHoldEnabled = 0;
		} else {
			fcStatusData.altitudeHoldEnabled = 1;
		}
	}
	float altDelta = altitudeData.altitudeSeaLevel - fcStatusData.altitudeRefSeaLevel;
	altDelta = constrainToRangeF(altDelta, -ALT_MGR_ALT_MAX_DISTANCE_DELTA, ALT_MGR_ALT_MAX_DISTANCE_DELTA);
	controlAltitudeWithGain(dt, altDelta, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f);
}

uint8_t stopAltitudeManager() {
	return 1;
}

uint8_t resetAltitudeManager() {
	altMgrThAggregationDt = 0;
	altMgrAltHoldActivationDt = 0;
	resetAltitudeControl(1);
	return 1;
}
