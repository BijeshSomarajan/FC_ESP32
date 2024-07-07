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
#include "attitudeSensor.h"
#include "imu.h"

TaskHandle_t wifiRcMgrTaskHandle;
int64_t wifiRcFCUpdateLastTimeUs;

#define WIFI_RC_MGR_FC_UPDATE_FREQUENCY 5.0f
#define WIFI_RC_MGR_FC_UPDATE_PERIOD 1.0f/WIFI_RC_MGR_FC_UPDATE_FREQUENCY
#define WIFI_RC_MGR_FC_UPDATE_PERIOD_US WIFI_RC_MGR_FC_UPDATE_PERIOD * 1000000.0f

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
		configureRCStickControl();
		wifiRcFCUpdateLastTimeUs = 0;
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

/**
 * canStart, canStabilize, canFly, hasCrashed, pitch        , roll           , yaw               , alt              , frequency
 * [0]     , [1]         , [2]   , [3]       , [4][5][6][7] , [8][9][10][11] , [12][13][14][15]  ,[16][17][18][19]  , [20][21][22][23]
 */
void getMarshalledFCState(uint8_t *outputData, uint16_t *outputLen) {
	int64_t currentTimeUs = getTimeUSec();
	if ((currentTimeUs - wifiRcFCUpdateLastTimeUs) >= WIFI_RC_MGR_FC_UPDATE_PERIOD_US) {
		wifiRcFCUpdateLastTimeUs = currentTimeUs;

		outputData[0] = fcStatusData.canStart;
		outputData[1] = fcStatusData.canArm;
		outputData[2] = fcStatusData.canStabilize;
		outputData[3] = fcStatusData.canFly;
		outputData[4] = fcStatusData.hasCrashed;

		int32_t scaledValue = imuData.pitch * 100;
		outputData[5] = scaledValue;
		outputData[6] = scaledValue >> 8;
		outputData[7] = scaledValue >> 16;
		outputData[8] = scaledValue >> 24;

		scaledValue = imuData.roll * 100;
		outputData[9] = scaledValue;
		outputData[10] = scaledValue >> 8;
		outputData[11] = scaledValue >> 16;
		outputData[12] = scaledValue >> 24;

		scaledValue = imuData.heading * 100;
		outputData[13] = scaledValue;
		outputData[14] = scaledValue >> 8;
		outputData[15] = scaledValue >> 16;
		outputData[16] = scaledValue >> 24;

		scaledValue = altitudeData.altitudeSeaLevel * 100;
		outputData[17] = scaledValue;
		outputData[18] = scaledValue >> 8;
		outputData[19] = scaledValue >> 16;
		outputData[20] = scaledValue >> 24;

		scaledValue = altitudeData.verticalVelocity * 100;
		outputData[21] = scaledValue;
		outputData[22] = scaledValue >> 8;
		outputData[23] = scaledValue >> 16;
		outputData[24] = scaledValue >> 24;

		scaledValue = 1.0f / imuData.dt;
		outputData[25] = scaledValue;
		outputData[26] = scaledValue >> 8;
		outputData[27] = scaledValue >> 16;
		outputData[28] = scaledValue >> 24;

		scaledValue = 1.0f / attitudeData.gyroDt;
		outputData[29] = scaledValue;
		outputData[30] = scaledValue >> 8;
		outputData[31] = scaledValue >> 16;
		outputData[32] = scaledValue >> 24;

		scaledValue = 1.0f / attitudeData.accDt;
		outputData[33] = scaledValue;
		outputData[34] = scaledValue >> 8;
		outputData[35] = scaledValue >> 16;
		outputData[36] = scaledValue >> 24;

		*outputLen = 37;
	} else {
		*outputLen = 0;
	}
}

uint8_t startRCManager() {
	BaseType_t err = xTaskCreatePinnedToCore(wifiRcManagerTask, "wifiRCManager", RC_WIFI_MANAGER_STACK_SIZE, NULL, RC_WIFI_MANAGER_PRIORITY, &wifiRcMgrTaskHandle, RC_WIFI_MANAGER_CPU);
	return (err == pdPASS);
}

void calibrateRC() {
	//No Impl
}

#endif
