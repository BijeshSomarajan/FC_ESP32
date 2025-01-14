#include "noiseSensor.h"
#include "cpuConfig.h"
#include "rcSensor.h"
#include "debugManager.h"
#include "fcStatus.h"
#include "control.h"
#include "lowPassFilter.h"
#include "imu.h"
#include "altitudeSensor.h"
#include "attitudeSensor.h"
#include "configSensor.h"
#include "deltaTimer.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fft.h"

TaskHandle_t debuggerTaskHandle;
int32_t DEBUG_DATA_BUFFER[8];

void debugTime() {
	DEBUG_DATA_BUFFER[0] = 1.0f / imuData.dt;
	DEBUG_DATA_BUFFER[1] = 1.0f / attitudeData.gyroDt;
	DEBUG_DATA_BUFFER[2] = 1.0f / attitudeData.accDt;
	DEBUG_DATA_BUFFER[3] = 1.0f / attitudeData.magDt;
	DEBUG_DATA_BUFFER[4] = 1.0f / attitudeData.tempDt;
	DEBUG_DATA_BUFFER[5] = 1.0f / altitudeData.dt;
	//DEBUG_DATA_BUFFER[6] = rcData.cpu;
	DEBUG_DATA_BUFFER[6] = 1.0f / rcData.readDt;
	sendConfigData(DEBUG_DATA_BUFFER, 7, CMD_FC_DATA);
}

void debugImu() {
	DEBUG_DATA_BUFFER[0] = imuData.linVz * 1000;
	DEBUG_DATA_BUFFER[1] = altitudeData.verticalVelocity * 100;
	DEBUG_DATA_BUFFER[2] = imuData.pitch * 10;
	DEBUG_DATA_BUFFER[3] = imuData.roll * 10;
	DEBUG_DATA_BUFFER[4] = imuData.heading;
	DEBUG_DATA_BUFFER[5] = 1.0f / imuData.dt;
	;
	DEBUG_DATA_BUFFER[6] = 1.0f / attitudeData.gyroDt;
	DEBUG_DATA_BUFFER[7] = 1.0f / attitudeData.accDt;
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void debugAttitude() {
	/*
	 DEBUG_DATA_BUFFER[0] = imuData.linAzGRaw * 1000;
	 DEBUG_DATA_BUFFER[1] = imuData.linVz * 1000;
	 DEBUG_DATA_BUFFER[2] = 1.0f / imuData.dt;
	 DEBUG_DATA_BUFFER[3] = 1.0f / attitudeData.gyroDt;
	 DEBUG_DATA_BUFFER[4] = 1.0f / attitudeData.accDt;
	 */
	DEBUG_DATA_BUFFER[0] = 1.0f / (10 * attitudeData.accDt);
	DEBUG_DATA_BUFFER[1] = 1.0f / (10 * attitudeData.gyroDt);
	DEBUG_DATA_BUFFER[2] = 1.0f / (10 * imuData.dt);
	DEBUG_DATA_BUFFER[3] = imuData.pitch * 10;
	DEBUG_DATA_BUFFER[4] = imuData.roll * 10;
	DEBUG_DATA_BUFFER[5] = imuData.heading;

	DEBUG_DATA_BUFFER[6] = imuData.rollRate * 10;
	DEBUG_DATA_BUFFER[7] = imuData.pitchRate * 10;
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void debugRC() {
	DEBUG_DATA_BUFFER[0] = fcStatusData.canStart * 10;
	DEBUG_DATA_BUFFER[1] = fcStatusData.canStabilize * 10;
	DEBUG_DATA_BUFFER[2] = fcStatusData.canFly * 10;
	DEBUG_DATA_BUFFER[3] = rcData.throttleCentered * 10;
	DEBUG_DATA_BUFFER[4] = fcStatusData.altitudeHoldEnabled * 10;
	DEBUG_DATA_BUFFER[5] = fcStatusData.currentThrottle;
	DEBUG_DATA_BUFFER[6] = rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX];
	DEBUG_DATA_BUFFER[7] = rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX];
	/*
	 DEBUG_DATA_BUFFER[6] = 1.0f / rcData.processDt;
	 DEBUG_DATA_BUFFER[7] = 1.0f / imuData.dt;
	 */
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void debugFCStatus() {
	DEBUG_DATA_BUFFER[0] = fcStatusData.canStart * 10;
	DEBUG_DATA_BUFFER[1] = fcStatusData.canStabilize * 10;
	DEBUG_DATA_BUFFER[2] = fcStatusData.canFly * 10;
	DEBUG_DATA_BUFFER[3] = rcData.throttleCentered;
	DEBUG_DATA_BUFFER[4] = fcStatusData.enableAltitudeHold;
	DEBUG_DATA_BUFFER[5] = rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX];
	DEBUG_DATA_BUFFER[6] = fcStatusData.hasCrashed * 10;
	DEBUG_DATA_BUFFER[7] = fcStatusData.hasTakenOff * 10;
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

extern LOWPASSFILTER altMgrThrottleControlLPF;
void debugAltitude() {
	DEBUG_DATA_BUFFER[0] = (altitudeData.altitudeSeaLevelCoarse - altitudeData.altitudeSeaLevelHome) * 10;
	DEBUG_DATA_BUFFER[1] = (altitudeData.altitudeSeaLevel - altitudeData.altitudeSeaLevelHome) * 10;
	DEBUG_DATA_BUFFER[2] = altitudeData.verticalVelocity * 100;
	DEBUG_DATA_BUFFER[3] = imuData.linVz * 1000;
	DEBUG_DATA_BUFFER[4] = controlData.altitudeControl * 10;
	DEBUG_DATA_BUFFER[5] = (controlData.altitudeControl + controlData.throttleControl) * 10;
	DEBUG_DATA_BUFFER[6] = altMgrThrottleControlLPF.output * 10;
	DEBUG_DATA_BUFFER[7] = fcStatusData.throttlePercentage * 10;
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void debugFFT() {

	 DEBUG_DATA_BUFFER[0] = attitudeData.gxDS * 100;
	 DEBUG_DATA_BUFFER[1] = attitudeData.gyDS * 100;
	 DEBUG_DATA_BUFFER[2] = attitudeData.gzDS * 100;

	 DEBUG_DATA_BUFFER[3] = attitudeData.axG * 1000;
	 DEBUG_DATA_BUFFER[4] = attitudeData.ayG * 1000;

	 DEBUG_DATA_BUFFER[5] = fftX.topFreq[0] ;
	 DEBUG_DATA_BUFFER[6] = fftX.topFreq[1];
	 DEBUG_DATA_BUFFER[7] = fftX.topFreq[2];
	/*
	DEBUG_DATA_BUFFER[0] = fftX.fftMagnitudes[1] * 10;
	DEBUG_DATA_BUFFER[1] = fftX.fftMagnitudes[2] * 10;
	DEBUG_DATA_BUFFER[2] = fftX.fftMagnitudes[3] * 10;
	DEBUG_DATA_BUFFER[3] = fftX.fftMagnitudes[4] * 10;
	DEBUG_DATA_BUFFER[4] = fftX.fftMagnitudes[5] * 10;
	DEBUG_DATA_BUFFER[5] = fftX.fftMagnitudes[6] * 10;
	DEBUG_DATA_BUFFER[6] = fftX.fftMagnitudes[7] * 10;
	DEBUG_DATA_BUFFER[7] = fftX.fftMagnitudes[8] * 10;
	*/
	/*
	 DEBUG_DATA_BUFFER[0] = fftX.topFreq[0];
	 DEBUG_DATA_BUFFER[1] = fftY.topFreq[0];
	 DEBUG_DATA_BUFFER[2] = fftX.fftMagnitudes[1] * 100;
	 DEBUG_DATA_BUFFER[3] = fftX.fftMagnitudes[2] * 100;
	 DEBUG_DATA_BUFFER[4] = fftX.fftMagnitudes[3] * 100;
	 DEBUG_DATA_BUFFER[5] = fftY.fftMagnitudes[1] * 100;
	 DEBUG_DATA_BUFFER[6] = fftY.fftMagnitudes[2] * 100;
	 DEBUG_DATA_BUFFER[7] = fftY.fftMagnitudes[3] * 100;
	 */
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

char printFFTbuf[512];
extern float fftBinFrequencies[FFT_HALF_N + 1];
void printFFT() {
	/*
	sprintf(printFFTbuf, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", fftX.fftMagnitudes[1], fftX.fftMagnitudes[2], fftX.fftMagnitudes[3], fftX.fftMagnitudes[4],
			fftX.fftMagnitudes[5], fftX.fftMagnitudes[6], fftX.fftMagnitudes[7], fftX.fftMagnitudes[8], fftX.fftMagnitudes[9], fftX.fftMagnitudes[10], fftX.fftMagnitudes[11], fftX.fftMagnitudes[12], fftX.fftMagnitudes[13], fftX.fftMagnitudes[14], fftX.fftMagnitudes[15], fftX.fftMagnitudes[16],
			fftX.fftMagnitudes[17], fftX.fftMagnitudes[18], fftX.fftMagnitudes[19], fftX.fftMagnitudes[20], fftX.fftMagnitudes[21], fftX.fftMagnitudes[22], fftX.fftMagnitudes[23], fftX.fftMagnitudes[24], fftX.fftMagnitudes[25], fftX.fftMagnitudes[26], fftX.fftMagnitudes[27], fftX.fftMagnitudes[28],
			fftX.fftMagnitudes[29], fftX.fftMagnitudes[30], fftX.fftMagnitudes[31], fftX.fftMagnitudes[32]);
	logString(printFFTbuf);
	*/

}

void doFcDebug(float dt) {
	//debugAltitude();
	//debugAttitude();
	//debugRC();
	//debugTime();
	//debugImu();
	debugFFT();
	//printFFT();
	//debugFCStatus();
}

int64_t debugMgrPrevTimeUs = 0;
int64_t debugMgrCurTimeUs = 0;
float debugMgrUpdateDt = 0;
void debuggerManagerTask(void *pvParameters) {
	debugMgrPrevTimeUs = getTimeUSec();
	float dt = 0;
	for (;;) {
		if (fcStatusData.debugEnabled) {
			debugMgrCurTimeUs = getTimeUSec();
			dt = getUSecTimeInSec(debugMgrCurTimeUs - debugMgrPrevTimeUs);
			debugMgrPrevTimeUs = debugMgrCurTimeUs;
			debugMgrUpdateDt += dt;
			if (debugMgrUpdateDt >= FC_DEBUG_PERIOD) {
				doFcDebug(debugMgrUpdateDt);
				debugMgrUpdateDt = 0;
			}
		}
		vTaskDelay(1);
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
//printf("--------- Debug[CPU= %d , Dt : %f] ---------\n", xPortGetCoreID(),dt);
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
