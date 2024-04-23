#include "debugManager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "deltaTimer.h"
#include "fcLogger.h"
#include "attitudeSensor.h"
#include "imu.h"
#include "rcSensor.h"
#include "pwmSensor.h"
#include "altitudeSensor.h"
#include "managerConfig.h"
#include "configSensor.h"
#include "fcStatus.h"
#include "rcManager.h"
#include "control.h"

TaskHandle_t debuggerTaskHandle;
float debugUpdateDt = 0;
int32_t DEBUG_DATA_BUFFER[8];

void debugTime() {
	DEBUG_DATA_BUFFER[0] = 1.0f / imuData.dt;
	DEBUG_DATA_BUFFER[1] = 1.0f / attitudeData.gyroDt;
	DEBUG_DATA_BUFFER[2] = 1.0f / attitudeData.accDt;
	DEBUG_DATA_BUFFER[3] = 1.0f / attitudeData.magDt;
	DEBUG_DATA_BUFFER[4] = 1.0f / attitudeData.tempDt;
	DEBUG_DATA_BUFFER[5] = 1.0f / altitudeData.dt;
	DEBUG_DATA_BUFFER[6] = 1.0f / rcData.processDt;
	DEBUG_DATA_BUFFER[7] = 1.0f / rcData.readDt;
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void debugImu() {
	DEBUG_DATA_BUFFER[0] = imuData.pitchRate * 10;
	DEBUG_DATA_BUFFER[1] = imuData.rollRate * 10;
	DEBUG_DATA_BUFFER[2] = imuData.yawRate * 10;

	DEBUG_DATA_BUFFER[3] = imuData.pitch * 10;
	DEBUG_DATA_BUFFER[4] = imuData.roll * 10;
	DEBUG_DATA_BUFFER[5] = imuData.heading;

	DEBUG_DATA_BUFFER[6] = imuData.linVz * 100;
	DEBUG_DATA_BUFFER[7] = 1.0f/imuData.dt;

	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void debugAttitude() {
	DEBUG_DATA_BUFFER[0] = controlData.pitchControl;
	DEBUG_DATA_BUFFER[1] = controlData.rollControl;
	DEBUG_DATA_BUFFER[2] = controlData.yawControl;

	DEBUG_DATA_BUFFER[3] = imuData.pitchRate * 100;
	DEBUG_DATA_BUFFER[4] = imuData.rollRate * 100;
	DEBUG_DATA_BUFFER[5] = imuData.yawRate * 100;

	DEBUG_DATA_BUFFER[6] = imuData.pitch * 10;
	DEBUG_DATA_BUFFER[7] = imuData.roll * 10;

	sendConfigData(DEBUG_DATA_BUFFER, 5, CMD_FC_DATA);
}

void debugAltitude() {
	DEBUG_DATA_BUFFER[0] = altitudeData.altitudeSeaLevelHome;
	DEBUG_DATA_BUFFER[1] = altitudeData.altitudeSeaLevelRaw;
	DEBUG_DATA_BUFFER[2] = altitudeData.altitudeSeaLevel;
	DEBUG_DATA_BUFFER[3] = fcStatusData.altitudeHoldEnabled * 100;
	DEBUG_DATA_BUFFER[4] = rcData.throttleCentered * 100;
	DEBUG_DATA_BUFFER[5] = altitudeData.verticalVelocity * 100;
	DEBUG_DATA_BUFFER[6] = controlData.altitudeControl * 100;
	DEBUG_DATA_BUFFER[7] = 1.0f / altitudeData.dt;
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void doFcDebug(float dt) {
	//debugAltitude();
	debugTime();
	//debugImu();
}

void debuggerManagerTask(void *pvParameters) {
	int64_t prevTimeUs = getTimeUSec();
	int64_t currentTimeUs = 0;
	float dt = 0;
	for (;;) {
		if (fcStatusData.debugEnabled) {
			currentTimeUs = getTimeUSec();
			dt = getUSecTimeInSec(currentTimeUs - prevTimeUs);
			debugUpdateDt += dt;
			if (debugUpdateDt >= FC_DEBUG_PERIOD) {
				doFcDebug(debugUpdateDt);
				debugUpdateDt = 0;
			}
		}
		vTaskDelay(100);
	}
}

uint8_t initDebugManager(void) {
	return 1;
}

uint8_t startDebugManager(void) {
	uint8_t status = 1;
	if (FC_DEBUG_ENABLED == 1) {
		BaseType_t err = xTaskCreatePinnedToCore(debuggerManagerTask, "deBugManager", DEBUGGER_STACK_SIZE, NULL, DEBUGGER_PRIORITY, &debuggerTaskHandle, DEBUGGER_CPU);
		status = (err == pdPASS);
	}
	return status;
}

//char deubugBuffer[500];
//printf("--------- Debug[CPU: %d , Dt : %f] ---------\n", xPortGetCoreID(),dt);
/*
 printf("Attitude Sensor [CPU: %d , Dt: %f ]\n", attitudeSensorData.cpu, attitudeSensorData.dt);
 printf("axG: %f, ayG: %f, azG: %f , gxDS: %f, gyDs: %f, gzDs: %f , T: %f\n", attitudeSensorData.axG, attitudeSensorData.ayG, attitudeSensorData.azG, attitudeSensorData.gxDS, attitudeSensorData.gyDS, attitudeSensorData.gzDS, attitudeSensorData.temp);
 printf("mx: %f, my: %f, mz: %f\n", attitudeSensorData.mx, attitudeSensorData.my, attitudeSensorData.mz);
 printf("IMU [CPU: %d , Dt: %f ]\n", imuData.cpu, imuData.dt);
 printf("pitch: %f, pitchRate: %f, roll: %f, rollRate: %f, heading: %f, yawRate: %f\n", imuData.pitch, imuData.pitchRate, imuData.roll,imuData.rollRate,imuData.heading,imuData.yawRate);
 */
/*sprintf(deubugBuffer, "DCPU:%d,IMUCPU:%d,AGCPU:%d,RCCPU:%d,PWMCPU:%d,ALTCPU:%d,IMU:%5.1f,AG:%5.1f,Mag:%5.1f,Tmp:%5.1f,RCUp:%5.1f,RCLd:%5.1f,PWM:%5.1f,ALT:%5.1f,ASL:%5.1f\n", xPortGetCoreID(), imuData.cpu, attitudeSensorData.cpu, rcData.cpu, pwmData.cpu, altitudeSensorData.cpu, 1.0f / imuData.dt,
 1.0f / attitudeSensorData.gyroDt, 1.0f / attitudeSensorData.magDt, 1.0f / attitudeSensorData.tempDt, 1.0f / rcData.updateDt, 1.0f / rcData.loadDt, 1.0f / pwmData.dt, 1.0f / altitudeSensorData.dt,altitudeSensorData.altitudeSeaLevel);
 */
//sprintf(deubugBuffer,"pitch: %f, pitchRate: %f, roll: %f, rollRate: %f, heading: %f, yawRate: %f\n", imuData.pitch, imuData.pitchRate, imuData.roll,imuData.rollRate,imuData.heading,imuData.yawRate);
//printf("RC  [CPU : %d , Dt: %f , Th : %d]\n", rcData.cpu, rcData.dt, rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX]);
//printf("PWM  [CPU : %d , Dt: %f , pwm[0] : %d , pwm[1] : %d , pwm[2] : %d , pwm[3] : %d\n", pwmData.cpu, pwmData.dt, pwmData.PWM_VALUES[0], pwmData.PWM_VALUES[1], pwmData.PWM_VALUES[2], pwmData.PWM_VALUES[3]);
//logString(deubugBuffer);
