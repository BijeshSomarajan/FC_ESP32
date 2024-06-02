#include "rcSensor.h"

#if WIFI_RC_SENSOR_ENABLED == 1

#include "fcWifi.h"
#include "delayTimer.h"
#include "deltaTimer.h"
#include "fcLogger.h"

//isOn,isLand,isRTH,th-value,pitch-value,roll-value,yaw-value
#define WIFI_RC_DATA_IS_ON_INDEX 0
#define WIFI_RC_DATA_IS_LAND_INDEX 1
#define WIFI_RC_DATA_IS_RTH_INDEX 2
#define WIFI_RC_DATA_IS_HEADLESS_INDEX 3
#define WIFI_RC_DATA_TH_DATA_INDEX 4
#define WIFI_RC_DATA_PITCH_DATA_INDEX 5
#define WIFI_RC_DATA_ROLL_DATA_INDEX 6
#define WIFI_RC_DATA_YAW_DATA_INDEX 7

uint16_t wifiRcChannelValue[RC_CHANNEL_COUNT] = { 0 };

extern void processRCData(float dt);
extern void determineFCState(void);
extern char* getWifiRCResponse(void);

int64_t wifiRcDataLoadLastTimeUs = 0;

void resetRcSensors() {
	//No Impl
}

uint8_t initRCSensors() {
	if (initWifi()) {
		logString("Wifi Rc Sensor , Init Success!\n");
	} else {
		logString("Wifi Rc Sensor , Init Failed!\n");
		return 0;
	}
	return 1;
}

uint8_t startRcSensors() {
	resetRcSensors();
	if (startHttpServer()) {
		logString("Wifi Rc Sensor , Start Success!\n");
	} else {
		logString("Wifi Rc Sensor , Start Failed!\n");
		return 0;
	}
	return 1;
}

void readRCSensors() {
	//No Impl
}

uint16_t getRCValue(uint8_t channel) {
	return (uint16_t) constrainToRange(wifiRcChannelValue[channel], RC_CHANNEL_MIN_VALUE, RC_CHANNEL_MAX_VALUE);
}

void calibrateRCSensor() {
	//No Impl
}



char* handleWifiPostData(char *data, int len) {
	int64_t currentTimeUs = getTimeUSec();
	float dt = getUSecTimeInSec(currentTimeUs - wifiRcDataLoadLastTimeUs);
	int curIndex = 0;
	char *token = strtok(data, ",");
	while (token != NULL) {
		if (curIndex == WIFI_RC_DATA_IS_ON_INDEX) {
			if (token[0] == '1') {
				wifiRcChannelValue[RC_START_CHANNEL_INDEX] = RC_CHANNEL_MAX_VALUE;
			} else {
				wifiRcChannelValue[RC_START_CHANNEL_INDEX] = RC_CHANNEL_MIN_VALUE;
			}
		} else if (curIndex == WIFI_RC_DATA_IS_LAND_INDEX) {
			if (token[0] == '1') {
				wifiRcChannelValue[RC_LAND_CHANNEL_INDEX] = RC_CHANNEL_MAX_VALUE;
			} else {
				wifiRcChannelValue[RC_LAND_CHANNEL_INDEX] = RC_CHANNEL_MIN_VALUE;
			}
		} else if (curIndex == WIFI_RC_DATA_IS_RTH_INDEX) {
			if (token[0] == '1') {
				wifiRcChannelValue[RC_POS_CHANNEL_INDEX] = RC_CHANNEL_MAX_VALUE;
			} else {
				wifiRcChannelValue[RC_POS_CHANNEL_INDEX] = RC_CHANNEL_MIN_VALUE;
			}
		} else if (curIndex == WIFI_RC_DATA_IS_HEADLESS_INDEX) {
			if (token[0] == '1') {
				wifiRcChannelValue[RC_HEADING_CHANNEL_INDEX] = RC_CHANNEL_MAX_VALUE;
			} else {
				wifiRcChannelValue[RC_HEADING_CHANNEL_INDEX] = RC_CHANNEL_MIN_VALUE;
			}
		} else if (curIndex == WIFI_RC_DATA_TH_DATA_INDEX) {
			wifiRcChannelValue[RC_TH_CHANNEL_INDEX] = atoi(token);
		} else if (curIndex == WIFI_RC_DATA_PITCH_DATA_INDEX) {
			wifiRcChannelValue[RC_PITCH_CHANNEL_INDEX] = atoi(token);
		} else if (curIndex == WIFI_RC_DATA_ROLL_DATA_INDEX) {
			wifiRcChannelValue[RC_ROLL_CHANNEL_INDEX] = atoi(token);
		} else if (curIndex == WIFI_RC_DATA_YAW_DATA_INDEX) {
			wifiRcChannelValue[RC_YAW_CHANNEL_INDEX] = atoi(token);
		} else {
			break;
		}
		token = strtok(NULL, ",");
		curIndex++;
	}
	processRCData(dt);
	determineFCState();
	return getWifiRCResponse();;
}

#endif
