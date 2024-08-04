#include "cpuConfig.h"
#include "pwmSensor.h"
#include "pwmManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "mathUtil.h"
#include "fcLogger.h"
#include "deltaTimer.h"
#include "delayTimer.h"
#include "rcManager.h"
#include "fcStatus.h"
#include "control.h"

TaskHandle_t pwmMgrTaskHandle;
int64_t updatePWMLastTimeUs = 0;

/**
 * Initializes the Radio Controller
 */
uint8_t initPWMManager() {
	if (!initPWMSensors()) {
		logString("PWM : Manager -> Initialization Failed\n");
		return 0;
	} else {
		pwmData.PWM_VALUES[0] = 0;
		pwmData.PWM_VALUES[1] = 0;
		pwmData.PWM_VALUES[2] = 0;
		pwmData.PWM_VALUES[3] = 0;
		updatePWMValues();
		logString("PWM : Manager -> Initialized\n");
	}
	return 1;
}

void updatePWMTask(void *arg) {
	int64_t currentTimeUs = getTimeUSec();
	float dt = getUSecTimeInSec(currentTimeUs - updatePWMLastTimeUs);
	updatePWMLastTimeUs = currentTimeUs;
	if (!fcStatusData.canFly) {
		pwmData.PWM_VALUES[0] = 0;
		pwmData.PWM_VALUES[1] = 0;
		pwmData.PWM_VALUES[2] = 0;
		pwmData.PWM_VALUES[3] = 0;
	} else {
		int throttle = controlData.throttleControl + controlData.altitudeControl;
		pwmData.PWM_VALUES[0] = throttle + controlData.pitchControl + controlData.rollControl + controlData.yawControl;
		pwmData.PWM_VALUES[1] = throttle + controlData.pitchControl - controlData.rollControl - controlData.yawControl;
		pwmData.PWM_VALUES[2] = throttle - controlData.pitchControl + controlData.rollControl - controlData.yawControl;
		pwmData.PWM_VALUES[3] = throttle - controlData.pitchControl - controlData.rollControl + controlData.yawControl;
	}
	updatePWMValues();
	pwmData.dt = dt;
}

const esp_timer_create_args_t pwmPeriodicTimerArgs = { .callback = &updatePWMTask, .name = "updatePWMTask" };
esp_timer_handle_t pwmPeriodicTimer;

uint8_t pwmManagerCreateTimers() {
	uint8_t status = 0;
	esp_err_t err = esp_timer_create(&pwmPeriodicTimerArgs, &pwmPeriodicTimer);
	if (err == ESP_OK) {
		logString("PWM Manager , Update Timer Created , Success!\n");
		status = 1;
	} else {
		logString("PWM Manager , Data Update Timer Created , Failed!\n");
	}
	return status;
}

uint8_t pwmManagerStartTimers() {
	uint8_t status = 0;
	esp_err_t err = esp_timer_start_periodic(pwmPeriodicTimer, PWM_UPDATE_PERIOD_US);
	if (err == ESP_OK) {
		logString("PWM Manager , Start Timers, Update Timer Start , Success!\n");
		status = 1;
	} else {
		logString("PWM Manager , Start Timers, Update Timer Start , Failed!\n");
	}
	return status;
}

void pwmManagerTask(void *pvParameters) {
	pwmData.cpu = xPortGetCoreID();
	if (pwmManagerCreateTimers()) {
		if (pwmManagerStartTimers()) {
			logString("PWM Manager , Timer Start success , Task running!\n");
			vTaskDelete(pwmMgrTaskHandle);
			/*
			for (;;) {
				vTaskDelay(1);
			}
			*/
		} else {
			logString("PWM Manager , Timer Start failed , Task Exiting!\n");
		}
	} else {
		logString("PWM Manager , Timer Creation failed , Task Exiting!\n");
	}
}

uint8_t startPWMManager() {
	if (!startPWMSensors()) {
		logString("PWM : Manager -> PWM Start Failed\n");
		return 0;
	}
	logString("PWM : Manager -> PWMMStarted\n");
	BaseType_t err = xTaskCreatePinnedToCore(pwmManagerTask, "pwmManager", PWM_MANAGER_STACK_SIZE, NULL, PWM_MANAGER_PRIORITY, &pwmMgrTaskHandle, PWM_MANAGER_CPU);
	return (err == pdPASS);
}

uint8_t stopPWMManager() {
	if (!stopPWMSensors()) {
		logString("PWM : Manager -> Stop Failed\n");
		return 0;
	} else {
		logString("PWM : Manager -> Stopped\n");
	}
	return 1;
}

