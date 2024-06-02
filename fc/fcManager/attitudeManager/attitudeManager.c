#include "rcSensor.h"
#include "attitudeManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "deltaTimer.h"
#include "fcLogger.h"
#include "attitudeSensor.h"
#include "imu.h"
#include "managerConfig.h"
#include "fcStatus.h"
#include "attitudeControl.h"
#include "control.h"

TaskHandle_t attitudeMgrTaskHandle;

int64_t updateIMULastTimeUs = 0;
int64_t readGyroLastTimeUs = 0;
int64_t readAccLastTimeUs = 0;
int64_t readMagLastTimeUs = 0;
int64_t readTempLastTimeUs = 0;
int64_t updateAttitudeControlLastTimeUs = 0;
float imuStabilizationDt = 0;
float crashThresholdG;
float lastThrottlePercentage = 0;

uint8_t initAttitudeManager() {
	uint8_t status = initAttitudeSensors(MOTOR_KV, BATTERY_VOLT, MOTOR_NUMBER);
	if (!status) {
		logString("init[sensor,attitude] > Failure\n");
		return 0;
	}
	logString("init[sensor,attitude] > Success\n");
	if (!initAttitudeControl()) {
		logString("init[control,attitude] > Failure\n");
		return 0;
	}
	logString("init[control,attitude] > Success\n");
	crashThresholdG = getMaxValidG();
	return 1;
}

void checkForCrash() {
	if (!fcStatusData.hasCrashed && fcStatusData.canFly) {
		if (fabs(attitudeData.azGRaw) >= crashThresholdG) {
			fcStatusData.hasCrashed = 1;
			fcStatusData.canFly = 0;
		} else {
			fcStatusData.hasCrashed = 0;
		}
	}
}

uint8_t stopAttitudeManager(void) {
	uint8_t status = 1;
	resetAttitudeControl(1);
	return status;
}

/**
 * Aligns Imu Data to Sensor
 */
void alignImuDataToBoard() {
	float temp = 0;
	imuData.pitch = -imuData.pitch;
	imuData.roll = -imuData.roll;
	temp = imuData.pitchRate;
	imuData.pitchRate = -imuData.rollRate;
	imuData.rollRate = -temp;
}

void updateIMUTask(void *arg) {
	int64_t currentTimeUs = getTimeUSec();
	float dt = getUSecTimeInSec(currentTimeUs - updateIMULastTimeUs);
	updateIMULastTimeUs = currentTimeUs;
	imuData.dt = dt;
	if (fcStatusData.canStabilize && !fcStatusData.isIMUStabilized) {
		imuStabilizationDt += dt;
		if (imuStabilizationDt <= ATTITUDE_STABILIZATION_PERIOD) {
			imuSetMode(1);
			imuUpdate(dt);
		} else {
			imuStabilizationDt = 0;
			fcStatusData.isIMUStabilized = 1;
			fcStatusData.headingRef = imuData.heading;
			imuSetMode(0);
		}
	} else if (fcStatusData.canFly) {
		imuUpdate(dt);
		alignImuDataToBoard();
		if (!rcData.yawCentered) {
			fcStatusData.headingRef = imuData.heading;
		}
	}
}

void readMagTask(void *arg) {
	if (!fcStatusData.isConfigMode) {
		int64_t currentTimeUs = getTimeUSec();
		float dt = getUSecTimeInSec(currentTimeUs - readMagLastTimeUs);
		readMagLastTimeUs = currentTimeUs;
		readMagSensor(dt);
		attitudeData.magDt = dt;
	}
}

void readTempTask(void *arg) {
	if (!fcStatusData.isConfigMode) {
		int64_t currentTimeUs = getTimeUSec();
		float dt = getUSecTimeInSec(currentTimeUs - readTempLastTimeUs);
		readTempLastTimeUs = currentTimeUs;
		readTempSensor(dt);
		attitudeData.tempDt = dt;
	}
}

#if SENSOR_READ_ACC_AND_GYRO_TOGETHER == 1
void readAccAndGyroTask(void *arg) {
	int64_t currentTimeUs = getTimeUSec();
	float dt = getUSecTimeInSec(currentTimeUs - readGyroLastTimeUs);
	readGyroLastTimeUs = currentTimeUs;
	if (!fcStatusData.isConfigMode) {
		readAccAndGyroSensor(dt);
		checkForCrash();
		attitudeData.gyroDt = dt;
		attitudeData.accDt = dt;
	}
}

const esp_timer_create_args_t readAccAndGyroPeriodicTimerArgs = { .callback = &readAccAndGyroTask, .name = "readAccAndGyroTask" };
esp_timer_handle_t readAccAndGyroPeriodicTimer;

uint8_t createAccAndGyroReadTimers() {
	esp_err_t err = esp_timer_create(&readAccAndGyroPeriodicTimerArgs, &readAccAndGyroPeriodicTimer);
	if (err == ESP_OK) {
		logString("Attitude Manager , Acc And Gyro Read Timer Created , Success!\n");
		return 1;
	} else {
		logString("Attitude Manager , Acc And Gyro Read Timer Creation , Success!\n");
	}
	return 0;
}

uint8_t startAccAndGyroReadTimers() {
	esp_err_t err = esp_timer_start_periodic(readAccAndGyroPeriodicTimer, SENSOR_GYRO_SAMPLE_PERIOD_US);
	if (err == ESP_OK) {
		logString("Attitude Manager , Start Timers, Read Acc and Gyro Start , Success!\n");
		return 1;
	} else {
		logString("Attitude Manager , Start Timers, Read Acc and Gyro Start , Failed!\n");
	}
	return 0;
}
#else

void readGyroTask(void *arg) {
	if (!fcStatusData.isConfigMode) {
		int64_t currentTimeUs = getTimeUSec();
		float dt = getUSecTimeInSec(currentTimeUs - readGyroLastTimeUs);
		readGyroLastTimeUs = currentTimeUs;
		readGyroSensor(dt);
		attitudeData.gyroDt = dt;
	}
}

void readAccTask(void *arg) {
	if (!fcStatusData.isConfigMode) {
		int64_t currentTimeUs = getTimeUSec();
		float dt = getUSecTimeInSec(currentTimeUs - readAccLastTimeUs);
		readAccLastTimeUs = currentTimeUs;
		readAccSensor(dt);
		checkForCrash();
		attitudeData.accDt = dt;
	}
}

const esp_timer_create_args_t readGyroPeriodicTimerArgs = { .callback = &readGyroTask, .name = "readGyroTask" };
esp_timer_handle_t readGyroPeriodicTimer;

const esp_timer_create_args_t readAccPeriodicTimerArgs = { .callback = &readAccTask, .name = "readAccTask" };
esp_timer_handle_t readAccPeriodicTimer;

uint8_t createAccAndGyroReadTimers() {
	esp_err_t err = esp_timer_create(&readGyroPeriodicTimerArgs, &readGyroPeriodicTimer);
	if (err == ESP_OK) {
		logString("Attitude Manager ,Gyro Read Timer Created , Success!\n");
		err = esp_timer_create(&readAccPeriodicTimerArgs, &readAccPeriodicTimer);
		if (err == ESP_OK) {
			logString("Attitude Manager , Acc Read Timer Created , Success!\n");
			return 1;
		} else {
			logString("Attitude Manager ,Acc Read Timer Creation , Failed!\n");
		}
	} else {
		logString("Attitude Manager ,Gyro Read Timer Creation , Failed!\n");
	}
	return 0;
}

uint8_t startAccAndGyroReadTimers() {
	esp_err_t err = esp_timer_start_periodic(readGyroPeriodicTimer, SENSOR_GYRO_SAMPLE_PERIOD_US);
	if (err == ESP_OK) {
		logString("Attitude Manager , Start Timers, Read Gyro Start , Success!\n");
		err = esp_timer_start_periodic(readAccPeriodicTimer, SENSOR_ACC_SAMPLE_PERIOD_US);
		if (err == ESP_OK) {
			logString("Attitude Manager , Start Timers, Read Acc Start , Success!\n");
			return 1;
		}
	}
	return 0;
}

#endif

void updateAttitudeControlTask(void *arg) {
	int64_t currentTimeUs = getTimeUSec();
	float dt = getUSecTimeInSec(currentTimeUs - updateAttitudeControlLastTimeUs);
	updateAttitudeControlLastTimeUs = currentTimeUs;
	if (!fcStatusData.isConfigMode && fcStatusData.canFly && fcStatusData.throttlePercentage > ATTITUDE_CONTROL_MIN_TH_PERCENT) {
		if (lastThrottlePercentage != fcStatusData.throttlePercentage) {
			lastThrottlePercentage = fcStatusData.throttlePercentage;
			calculateMotorNoise(fcStatusData.throttlePercentage);
		}

		float expectedPitch = (float) rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX];
		float expectedRoll = -(float) rcData.RC_EFFECTIVE_DATA[RC_ROLL_CHANNEL_INDEX];
		float expectedYaw = (float) rcData.RC_EFFECTIVE_DATA[RC_YAW_CHANNEL_INDEX];
		expectedPitch = constrainToRangeF(expectedPitch, -ATT_MAX_PITCH_ROLL, ATT_MAX_PITCH_ROLL);
		expectedRoll = constrainToRangeF(expectedRoll, -ATT_MAX_PITCH_ROLL, ATT_MAX_PITCH_ROLL);

		controlAttitude(dt, expectedPitch, expectedRoll, expectedYaw, 1.0f, 1.0f, 1.0f, 1.0f);
		controlData.altitudeControlDt = dt;
	} else {
		lastThrottlePercentage = fcStatusData.throttlePercentage;
		calculateMotorNoise(fcStatusData.throttlePercentage);
		resetAttitudeControl(1);

	}
}

const esp_timer_create_args_t imuPeriodicTimerArgs = { .callback = &updateIMUTask, .name = "updateIMUTask" };
esp_timer_handle_t imuPeriodicTimer;

const esp_timer_create_args_t readMagPeriodicTimerArgs = { .callback = &readMagTask, .name = "readMagTask" };
esp_timer_handle_t readMagPeriodicTimer;

const esp_timer_create_args_t readTempPeriodicTimerArgs = { .callback = &readTempTask, .name = "readTempTask" };
esp_timer_handle_t readTempPeriodicTimer;

const esp_timer_create_args_t altControlPeriodicTimerArgs = { .callback = &updateAttitudeControlTask, .name = "altcontrol" };
esp_timer_handle_t altControlPeriodicTimer;

uint8_t attitudeManagerCreateTimers() {
	uint8_t status = 0;
	esp_err_t err = esp_timer_create(&imuPeriodicTimerArgs, &imuPeriodicTimer);
	if (err == ESP_OK) {
		logString("Attitude Manager , IMU Timer Created , Success!\n");
		if (createAccAndGyroReadTimers()) {
			logString("Attitude Manager , Acc and Gyro Read Timer Creation , Success!\n");
			err = esp_timer_create(&readMagPeriodicTimerArgs, &readMagPeriodicTimer);
			if (err == ESP_OK) {
				logString("Attitude Manager , Mag Read Timer Created , Success!\n");
				err = esp_timer_create(&readTempPeriodicTimerArgs, &readTempPeriodicTimer);
				if (err == ESP_OK) {
					logString("Attitude Manager , Temp Read Timer Created , Success!\n");
					err = esp_timer_create(&altControlPeriodicTimerArgs, &altControlPeriodicTimer);
					if (err == ESP_OK) {
						logString("Attitude Manager , Alt control Read Timer Created , Success!\n");
						status = 1;
					} else {
						logString("Attitude Manager , Alt control Read Timer Creation, Failed!\n");
					}
				} else {
					logString("Attitude Manager , Temp Read Timer Created , Failed!\n");
				}
			} else {
				logString("Attitude Manager , Mag Read Timer Created , Failed!\n");
			}
		} else {
			logString("Attitude Manager , Acc and Gyro Read Timer Creation , Failed!\n");
		}
	} else {
		logString("Attitude Manager , IMU Timer Created , Failed!\n");
	}
	return status;
}

uint8_t attitudeManagerStartTimers() {
	uint8_t status = 0;
	esp_err_t err = esp_timer_start_periodic(imuPeriodicTimer, ATTITUDE_IMU_UPDATE_PERIOD_US);
	if (err == ESP_OK) {
		logString("Attitude Manager , Start Timers, IMU Update Timer Start , Success!\n");
		if (startAccAndGyroReadTimers()) {
			logString("Attitude Manager , Start Timers, Read Acc and Gyro  , Success!\n");
			err = esp_timer_start_periodic(readMagPeriodicTimer, SENSOR_MAG_SAMPLE_PERIOD_US);
			if (err == ESP_OK) {
				logString("Attitude Manager , Start Timers, Read Mag Start , Success!\n");
				err = esp_timer_start_periodic(readTempPeriodicTimer, SENSOR_TEMP_SAMPLE_PERIOD_US);
				if (err == ESP_OK) {
					logString("Attitude Manager , Start Timers, Read Temp Start , Success!\n");
					err = esp_timer_start_periodic(altControlPeriodicTimer, ATTITUDE_CONTROL_UPDATE_PERIOD_US);
					if (err == ESP_OK) {
						logString("Attitude Manager , Start Timers, Alt control Start , Success!\n");
						status = 1;
					} else {
						logString("Attitude Manager , Start Timers, Alt control Start , Failed!\n");
					}
				} else {
					logString("Attitude Manager , Start Timers, Read Temp Start , Failed!\n");
				}
			} else {
				logString("Attitude Manager , Start Timers, Read Mag Failed , Success!\n");
			}
		} else {
			logString("Attitude Manager , Start Timers, Read Acc and Gyro  , Failed!\n");
		}
	} else {
		logString("Attitude Manager , Start Timers, IMU Update Timer Start , Failed!\n");
	}
	return status;
}

void attitudeManagerTask(void *pvParameters) {
	attitudeData.cpu = xPortGetCoreID();
	if (attitudeManagerCreateTimers()) {
		if (attitudeManagerStartTimers()) {
			logString("Attitude Manager , Timer Start success , Task running!\n");
			/*
			 for (;;) {
			 vTaskDelay(1);
			 }
			 */
			vTaskDelete(attitudeMgrTaskHandle);
		} else {
			logString("Attitude Manager , Timer Start failed , Task Exiting!\n");
		}
	} else {
		logString("Attitude Manager , Timer Creation failed , Task Exiting!\n");
	}
}

uint8_t startAttitudeManager(void) {
	uint8_t status = imuInit(MAG_INCLINATION);
	if (status) {
		BaseType_t err = xTaskCreatePinnedToCore(attitudeManagerTask, "AttitudeManager", ATTITUDE_MANAGER_STACK_SIZE, NULL, ATTITUDE_MANAGER_PRIORITY, &attitudeMgrTaskHandle, ATTITUDE_MANAGER_CPU);
		status = (err == pdPASS);
	}
	return status;
}

void calibarateAGForBias() {
	calculateAccAndGyroBias();
}

void calibarateMagForBias() {
	calculateMagBias();
}

void calibarateAccAndGyroForTemp() {
	calculateAccAndGyroTempCoeff();
}

uint8_t resetAttitudeManager() {
	resetAttitudeControl(1);
	return 1;
}

