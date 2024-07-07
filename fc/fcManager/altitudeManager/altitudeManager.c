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
float altStabilizationDt = 0;

LOWPASSFILTER altMgrThrottleControlLPF;
LOWPASSFILTER altMgrVelocityLPF;

float altMgrRateKDGain = 0;
float altMgrKIGain = 0;
float altMgrKPGain = 0;

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
	altMgrKPGain = ALT_MGR_CONTROL_P_GAIN_MIN;
	altMgrKIGain = ALT_MGR_CONTROL_I_GAIN_MIN;
	altMgrRateKDGain = ALT_MGR_CONTROL_RATE_D_GAIN_MIN;
	initAltitudeControl();
	logString("init[Controller,altitude] > Success\n");
	logString("init[Manager,altitude] > Success\n");
	return 1;
}

#if ALTITUDE_MGR_USE_TIMER == 1

int64_t altitudeUpdateLastTimeUs = 0;
void updateAltitudeDataTask(void *pvParameters) {
	if (!fcStatusData.isConfigMode) {
		altitudeData.cpu = xPortGetCoreID();
		int64_t currentTimeUs = getTimeUSec();
		float dt = getUSecTimeInSec(currentTimeUs - altitudeUpdateLastTimeUs);
		altitudeUpdateLastTimeUs = currentTimeUs;
		altitudeData.dt = dt;
		readAltitudeSensors(dt);
		if (fcStatusData.canStabilize && !fcStatusData.isAltStabilized) {
			altStabilizationDt += dt;
			if (altStabilizationDt > ALTITUDE_STABILIZATION_PERIOD) {
				altStabilizationDt = 0;
				fcStatusData.isAltStabilized = 1;
				altitudeData.altitudeSeaLevelHome = altitudeData.altitudeSeaLevel;
				fcStatusData.altitudeRefSeaLevel = altitudeData.altitudeSeaLevel;
			}
			resetAltitudeControl(1);
			fcStatusData.throttlePercentage = 0;
			controlData.throttleControl = 0;
			fcStatusData.currentThrottle = 0;
			altMgrThAggregationDt = 0;
			lowPassFilterReset(&altMgrThrottleControlLPF);
		} else if (fcStatusData.canFly) {
			manageAltitude(dt);
		} else {
			lowPassFilterReset(&altMgrThrottleControlLPF);
			resetAltitudeControl(1);
			fcStatusData.throttlePercentage = 0;
			controlData.throttleControl = 0;
			fcStatusData.currentThrottle = 0;
			altMgrThAggregationDt = 0;
		}
	}
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
	if (altitudeManagerCreateTimers()) {
		if (altitudeManagerStartTimers()) {
			logString("Altitude Manager , Timer Start success , Task running!\n");
			vTaskDelete(altitudeMgrTaskHandle);
			/*
			 for (;;) {
			 vTaskDelay(1);
			 }*/
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
	logString("Altitude Manager Task, Stabilized Altitude!\n");
	for (;;) {
		currentTimeUs = getTimeUSec();
		dt = getUSecTimeInSec(currentTimeUs - prevTimeUs);
		prevTimeUs = currentTimeUs;
		altitudeUpdateDt += dt;
		if (altitudeUpdateDt >= SENSOR_ALT_SAMPLE_PERIOD) {
			readAltitudeSensors(altitudeUpdateDt);
			altitudeData.dt = altitudeUpdateDt;
			if (fcStatusData.canStabilize && !fcStatusData.isAltStabilized) {
					altStabilizationDt += altitudeUpdateDt;
					if (altStabilizationDt > ALTITUDE_STABILIZATION_PERIOD) {
						altStabilizationDt = 0;
						fcStatusData.isAltStabilized = 1;
						altitudeData.altitudeSeaLevelHome = altitudeData.altitudeSeaLevel;
						fcStatusData.altitudeRefSeaLevel = altitudeData.altitudeSeaLevel;
					}
					controlData.throttleControl = 0;
					controlData.altitudeControl = 0;
					fcStatusData.throttlePercentage = 0;
				} else if (fcStatusData.canFly) {
					manageAltitude(altitudeUpdateDt);
				}else {
					fcStatusData.throttlePercentage = 0;
					controlData.throttleControl = 0;
					controlData.altitudeControl = 0;
				}
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
	float altitudeV = applyDeadBandFloat(0.0f, (imuData.linVz) * ALT_MGR_VERTICAL_VELOCITY_INPUT_GAIN, ALT_MGR_VERTICAL_VELOCITY_DEADBAND);
	altitudeData.verticalVelocity = altitudeV * ALT_MGR_VERTICAL_VELOCITY_OUTPUT_GAIN;
	altitudeData.verticalVelocity = constrainToRangeF(altitudeData.verticalVelocity, -ALT_MGR_VERTICAL_VELOCITY_MAX, ALT_MGR_VERTICAL_VELOCITY_MAX);
	altitudeData.verticalVelocity = lowPassFilterUpdate(&altMgrVelocityLPF, altitudeData.verticalVelocity, dt);
}

void manageAltitude(float dt) {
	calculateAltVelocity(dt);
	if (!rcData.throttleCentered || fcStatusData.throttlePercentage < ALT_MGR_CONTROL_MIN_TH_PERCENT) {
		//If Alt hold was enabled take the LPF output
		if (fcStatusData.altitudeHoldEnabled) {
			fcStatusData.currentThrottle = altMgrThrottleControlLPF.output;
		}
		aggregateThrottle(dt);
		syncToAltCoarseValue();
		resetAltitudeControl(1);
		controlData.throttleControl = fcStatusData.currentThrottle;
		fcStatusData.altitudeRefSeaLevel = altitudeData.altitudeSeaLevel;
		altMgrAltHoldActivationDt = 0;
		fcStatusData.altitudeHoldEnabled = 0;
		altMgrKPGain = ALT_MGR_CONTROL_P_GAIN_MIN;
		altMgrKIGain = ALT_MGR_CONTROL_I_GAIN_MIN;
		altMgrRateKDGain = ALT_MGR_CONTROL_RATE_D_GAIN_MIN;
	} else {
		if (altMgrAltHoldActivationDt <= ALT_MGR_ALT_HOLD_ACTIVATION_PERIOD) {
			altMgrAltHoldActivationDt += dt;
			syncToAltCoarseValue();
			fcStatusData.altitudeRefSeaLevel = altitudeData.altitudeSeaLevel;
			fcStatusData.altitudeHoldEnabled = 0;
			altMgrKPGain = ALT_MGR_CONTROL_P_GAIN_MIN;
			altMgrKIGain = ALT_MGR_CONTROL_I_GAIN_MIN;
			altMgrRateKDGain = ALT_MGR_CONTROL_RATE_D_GAIN_MIN;
			resetAltitudeControl(1);
		} else {
			if (altMgrKPGain < ALT_MGR_CONTROL_P_GAIN_MAX) {
				altMgrKPGain += ALT_MGR_CONTROL_P_GAIN_FACTOR;
			} else {
				altMgrKPGain = ALT_MGR_CONTROL_P_GAIN_MAX;
			}
			if (altMgrKIGain < ALT_MGR_CONTROL_I_GAIN_MAX) {
				altMgrKIGain += ALT_MGR_CONTROL_I_GAIN_FACTOR;
			} else {
				altMgrKIGain = ALT_MGR_CONTROL_I_GAIN_MAX;
			}
			if (altMgrRateKDGain < ALT_MGR_CONTROL_RATE_D_GAIN_MAX) {
				altMgrRateKDGain += ALT_MGR_CONTROL_RATE_D_GAIN_FACTOR;
			} else {
				altMgrRateKDGain = ALT_MGR_CONTROL_RATE_D_GAIN_MAX;
			}
			fcStatusData.altitudeHoldEnabled = 1;
		}
		float altDelta = altitudeData.altitudeSeaLevel - fcStatusData.altitudeRefSeaLevel;
		altDelta = constrainToRangeF(altDelta, -ALT_MGR_ALT_MAX_DISTANCE_DELTA, ALT_MGR_ALT_MAX_DISTANCE_DELTA);

#if ALT_MGR_ALT_DETLA_SQRT_ENABLE == 1
		if (altDelta > 0.0f) { // This check is required else take fabs
			altDelta = sqrtf(altDelta);
		}
#endif
		controlAltitudeWithGain(dt, altDelta, 0.0f, altMgrKPGain, altMgrKIGain, 1.0f, altMgrRateKDGain);
	}
	lowPassFilterUpdate(&altMgrThrottleControlLPF, controlData.throttleControl + controlData.altitudeControl, dt);
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
