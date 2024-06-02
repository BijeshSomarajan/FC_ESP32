#include "rcManager.h"

#if WIFI_RC_MANAGER_ENABLED == 1

#include "rcSensor.h"
#include "mathUtil.h"
#include "fcLogger.h"
#include "managerConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "mathUtil.h"
#include "fcLogger.h"
#include "delayTimer.h"
#include "deltaTimer.h"
#include "fcStatus.h"
#include "altitudeSensor.h"
#include "imu.h"


TaskHandle_t wifiRcMgrTaskHandle;

void loadRCStickMids() {
	for (uint8_t indx = 0; indx < RC_CHANNEL_COUNT; indx++) {
		rcData.RC_MID_DATA[indx] = RC_CHANNEL_MIN_VALUE;
	}
	rcData.RC_MID_DATA[RC_TH_CHANNEL_INDEX] = RC_CHANNEL_MID_VALUE;
	rcData.RC_MID_DATA[RC_PITCH_CHANNEL_INDEX] = RC_CHANNEL_MID_VALUE;
	rcData.RC_MID_DATA[RC_ROLL_CHANNEL_INDEX] = RC_CHANNEL_MID_VALUE;
	rcData.RC_MID_DATA[RC_YAW_CHANNEL_INDEX] = RC_CHANNEL_MID_VALUE;
}

uint8_t initRCManager() {
	if (initRCSensors()) {
		loadRCStickMids();
		logString("Wifi Rc Manager , Init Success!\n");
	} else {
		logString("Wifi Rc Manager , Init Failed!\n");
		return 0;
	}
	return 1;
}

void wifiRcManagerTask(void *pvParameters) {
	if (startRcSensors()) {
		logString("Wifi Rc Manager , Start Success!\n");
		rcData.cpu = xPortGetCoreID();
		vTaskDelete(wifiRcMgrTaskHandle);
	} else {
		logString("Wifi Rc Manager , Start Failed!\n");
	}
}

uint8_t startRCManager() {
	BaseType_t err = xTaskCreatePinnedToCore(wifiRcManagerTask, "wifiRCManager", RC_WIFI_MANAGER_STACK_SIZE, NULL, RC_WIFI_MANAGER_PRIORITY, &wifiRcMgrTaskHandle, RC_WIFI_MANAGER_CPU);
	return (err == pdPASS);
}

void calibrateRC() {
	//No Impl
}

char fcStusResponse[128];
char* getWifiRCResponse() {
	sprintf(fcStusResponse, "%d,%d,%d,%d,%6.2f,%6.2f,%6.2f,%7.2f,%7.2f", fcStatusData.canStart, fcStatusData.canStabilize, fcStatusData.canFly, fcStatusData.hasCrashed, altitudeData.altitudeSeaLevel, imuData.pitch, imuData.roll, imuData.heading,1.0f/imuData.dt);
	return fcStusResponse;
}

#endif
