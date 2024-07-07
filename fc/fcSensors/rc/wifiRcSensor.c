#include "rcSensor.h"

#if WIFI_RC_SENSOR_ENABLED == 1

#include "fcWifi.h"
#include "fcWifiIO.h"
#include "delayTimer.h"
#include "deltaTimer.h"
#include "fcLogger.h"

uint16_t wifiRcChannelValue[RC_CHANNEL_COUNT] = { 0 };

extern void processRCData(float dt);
extern void determineFCState(void);
extern void getMarshalledFCState(uint8_t *outputData, uint16_t *outputLen);

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
	rcData.readDt = 0;
	return 1;
}

uint8_t startRcSensors() {
	resetRcSensors();
	if (startUDPServer()) {
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

/*
 On/Off, Land, RTH, Throtte, pitch,  roll  , yaw
 [0], [1] , [2], [3][4] , [5][6], [7][8], [9][10]
 */
void unMarshallWifiData(uint8_t *inputData, int inputLen) {
	if (inputData[0] == 1) {
		wifiRcChannelValue[RC_START_CHANNEL_INDEX] = RC_CHANNEL_MAX_VALUE;
	} else {
		wifiRcChannelValue[RC_START_CHANNEL_INDEX] = RC_CHANNEL_MIN_VALUE;
	}
	if (inputData[1] == 1) {
		wifiRcChannelValue[RC_LAND_CHANNEL_INDEX] = RC_CHANNEL_MAX_VALUE;
	} else {
		wifiRcChannelValue[RC_LAND_CHANNEL_INDEX] = RC_CHANNEL_MIN_VALUE;
	}
	if (inputData[2] == 1) {
		wifiRcChannelValue[RC_POS_CHANNEL_INDEX] = RC_CHANNEL_MAX_VALUE;
	} else {
		wifiRcChannelValue[RC_POS_CHANNEL_INDEX] = RC_CHANNEL_MIN_VALUE;
	}
	int16_t tempData;
	//Throttle
	tempData = inputData[3];
	tempData |= (uint16_t) (inputData[4] << 8);
	wifiRcChannelValue[RC_TH_CHANNEL_INDEX] = tempData;
	//Pitch
	tempData = inputData[5];
	tempData |= (uint16_t) (inputData[6] << 8);
	wifiRcChannelValue[RC_PITCH_CHANNEL_INDEX] = tempData;
	//Roll
	tempData = inputData[7];
	tempData |= (uint16_t) (inputData[8] << 8);
	wifiRcChannelValue[RC_ROLL_CHANNEL_INDEX] = tempData;
	//Yaw
	tempData = inputData[9];
	tempData |= (uint16_t) (inputData[10] << 8);
	wifiRcChannelValue[RC_YAW_CHANNEL_INDEX] = tempData;


	/*
	char tempDebug[100];
	sprintf(tempDebug, "Start:%d, Land:%d, Pos:%d, Th:%d, Pitch:%d, Roll:%d, Yaw:%d\n", wifiRcChannelValue[RC_START_CHANNEL_INDEX], wifiRcChannelValue[RC_LAND_CHANNEL_INDEX], wifiRcChannelValue[RC_POS_CHANNEL_INDEX], wifiRcChannelValue[RC_TH_CHANNEL_INDEX],
			wifiRcChannelValue[RC_PITCH_CHANNEL_INDEX], wifiRcChannelValue[RC_ROLL_CHANNEL_INDEX], wifiRcChannelValue[RC_YAW_CHANNEL_INDEX]);
	logString(tempDebug);
	*/
}

void handleWifiRxData(uint8_t *inputData, int inputLen, uint8_t *outputData, uint16_t *outputLen) {
	int64_t currentTimeUs = getTimeUSec();
	float dt = getUSecTimeInSec(currentTimeUs - wifiRcDataLoadLastTimeUs);
	wifiRcDataLoadLastTimeUs = currentTimeUs;
	unMarshallWifiData(inputData, inputLen);
	rcData.readDt = dt;
	processRCData(dt);
	determineFCState();
	getMarshalledFCState(outputData, outputLen);
}

#endif
