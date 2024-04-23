#include "configManager.h"

#include "indicatorSensor.h"
#include "calibration.h"
#include "fcLogger.h"
#include "managerConfig.h"
#include "configSensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "deltaTimer.h"
#include "fcUART.h"
#include "fcStatus.h"
#include "attitudeManager.h"
#include "rcManager.h"

TaskHandle_t configMgrTaskHandle;
float configDataReadDt = 0;
float configDataProcessDt = 0;

int32_t FC_CONFIG_DATA_BUFFER[CONFIG_DATA_MAX_LENGTH];

uint8_t initConfigManager() {
	uint8_t status = initCalibration();
	if (status) {
		logString("init: Configuration -> Initialized\n");
		//Initialize with default calibration
		setDefaultCalibration();
		//Checking calibration
		if (!isCalibrated()) {
			//Save default calibration
			status = saveCalibration();
			if (status) {
				logString("init: Configuration -> Default , Success\n");
			} else {
				logString("init: Calibration -> Default , Failed!\n");
			}
		} else {
			logString("init: Configuration -> Already Configured!\n");
			//Load the persisted calibrations
			status = loadCalibration();
			if (status) {
				logString("init: Configuration -> Loaded\n");
			} else {
				logString("init: Configuration -> failed\n");
			}
		}
		if (status) {
			logString("Configuration sensors initializing\n");
			status = initConfigSensors();
			if (status) {
				logString("Configuration sensors Init -> Success\n");
			} else {
				logString("Configuration sensors Init -> Failure\n");
			}
		}
	} else {
		logString("init: Calibration -> Initialization Failed\n");
	}
	return status;
}

/**
 * Checks if there is a new configuration
 */
uint8_t hasNewConfiguration() {
	return hasConfigDataPacket();
}


/**
 *Digests the available data packets
 */
void manageConfigDataPacket() {
	ConfigDataPacket dataPacket = getConfigDataPacket();
	fcStatusData.debugEnabled = 0;
	fcStatusData.isConfigMode = 1;
	if (dataPacket.cmd == CMD_START_FC_DATA) {
		fcStatusData.debugEnabled = 1;
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_START_FC_DATA);
		fcStatusData.isConfigMode = 0;
	} else if (dataPacket.cmd == CMD_STOP_FC_DATA) {
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_STOP_FC_DATA);
		fcStatusData.isConfigMode = 0;
	} else if (dataPacket.cmd == CMD_READ_CONFIG) {
		sendConfigData(getCalibrationData(), CALIB_PROP_MAX_CONFIGURABLE_LENGTH, CMD_ACK_READ_CONFIG);
		fcStatusData.isConfigMode = 0;
	} else if (dataPacket.cmd == CMD_SAVE_CONFIG) {
		for (uint32_t indx1 = 0; indx1 < CALIB_PROP_MAX_CONFIGURABLE_LENGTH; indx1++) {
			setCalibrationValue(indx1, dataPacket.data[indx1]);
		}
		if (saveCalibration()) {
			sendConfigData(getCalibrationData(), CALIB_PROP_MAX_CONFIGURABLE_LENGTH, CMD_ACK_SAVE_CONFIG);
		}
		fcStatusData.isConfigMode = 0;
	} else if (dataPacket.cmd == CMD_CALIBRATE_IMU_OFFSET) {
		calibarateAGForBias();
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_CALIBRATE_IMU_OFFSET);
	} else if (dataPacket.cmd == CMD_CALIBRATE_IMU_TEMP) {
		calibarateAccAndGyroForTemp();
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_CALIBRATE_IMU_TEMP);
	} else if (dataPacket.cmd == CMD_CALIBRATE_IMU_MAG) {
		calibarateMagForBias();
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_CALIBRATE_IMU_MAG);
	} else if (dataPacket.cmd == CMD_CALIBRATE_RC) {
		calibrateRC();
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_CALIBRATE_RC);
	}
}

void configManagerTask(void *pvParameters) {
//	rcData.cpu = xPortGetCoreID();
	int64_t prevTimeUs = getTimeUSec();
	int64_t currentTimeUs = 0;
	float dt = 0;
	for (;;) {
		currentTimeUs = getTimeUSec();
		dt = getUSecTimeInSec(currentTimeUs - prevTimeUs);
		prevTimeUs = currentTimeUs;
		configDataReadDt += dt;
		configDataProcessDt += dt;
		if (configDataReadDt >= CONFIG_DATA_READ_PERIOD) {
			readConfigSensors();
			configDataReadDt = 0;
		}
		if (configDataProcessDt >= CONFIG_DATA_PROCESS_PERIOD) {
			if (hasNewConfiguration()) {
				statusIndicatorSensorOn();
				manageConfigDataPacket();
				statusIndicatorSensorOff();
			}
			configDataProcessDt = 0;
		}
		if (fcStatusData.isConfigMode) {
			vTaskDelay(0);
		} else {
			vTaskDelay(1);
		}
	}
}

uint8_t startConfigManager(void) {
	BaseType_t err = xTaskCreatePinnedToCore(configManagerTask, "configManager", CONFIG_MANAGER_STACK_SIZE, NULL, CONFIG_MANAGER_PRIORITY, &configMgrTaskHandle, CONFIG_MANAGER_CPU);
	return (err == pdPASS);
}
