#include "cpuConfig.h"
#include "rcSensor.h"
#include "fcLogger.h"
#include "deltaTimer.h"
#include "pwmSensor.h"
#include "pwmManager.h"
#include "attitudeSensor.h"
#include "fcManager.h"
#include "indicatorSensor.h"
#include "fcStatus.h"
#include "attitudeManager.h"
#include "altitudeManager.h"
#include "debugManager.h"
#include "configManager.h"
#include "rcManager.h"

uint8_t initManagers() {
	if (!initConfigManager()) {
		logString("init[manager,config] > Failure\n");
		return 0;
	}
	logString("init[manager,config] > Success\n");

	if (!initRCManager()) {
		logString("init[manager,RC] > Failure\n");
		return 0;
	}
	logString("init[manager,RC] > Success\n");

	if (!initPWMManager()) {
		logString("init[manager,PWM] > Failure\n");
		return 0;
	}
	logString("init[manager,PWM] > Success\n");

	if (!initAttitudeManager()) {
		logString("init[manager , AttitudeManager] > Failure\n");
		return 0;
	}
	logString("init[manager , AttitudeManager] > Success\n");

	if (!initAltitudeManager()) {
		logString("init[manager , AltitudeManager] > Failure\n");
		return 0;
	}
	logString("init[manager , AltitudeManager] > Success\n");

	if (!initDebugManager()) {
		logString("init[manager , DebugManager] > Failure\n");
		return 0;
	}
	logString("init[manager , DebugManager] > Success\n");

	return 1;
}

uint8_t initFlightController() {
	if (!initManagers()) {
		logString("init[managers] > Failure\n");
		return 0;
	}
	logString("init[managers] > Success\n");
	return 1;
}

uint8_t startFlightTasks() {
	if (!startRCManager()) {
		logString("init[Start , RCManager] > Failure\n");
		return 0;
	}
	if (!startAttitudeManager()) {
		logString("init[Start , AttitudeManager] > Failure\n");
		return 0;
	}
	logString("init[Start , AttitudeManager] > Success\n");

	if (!startAltitudeManager()) {
		logString("init[Start , AltitudeManager] > Failure\n");
		return 0;
	}
	logString("init[Start , AltitudeManager] > Success\n");
	if (!startPWMManager()) {
		logString("init[Start , PWMManager] > Failure\n");
		return 0;
	}
	logString("init[Start , PWMManager] > Success\n");
	if (!startDebugManager()) {
		logString("init[Start , DebugManager] > Failure\n");
		return 0;
	}
	if (!startConfigManager()) {
		logString("init[Start , ConfigManager] > Failure\n");
		return 0;
	}
	logString("init[Start , ConfigManager] > Success\n");

	return 1;
}

void startFlightController() {
	if (initLogger()) {
		logString("Starting : FC\n");
		if (initIndicatorSensors()) {
			logString("init[Indicator Sensors] > Success\n");
			if (initFlightController()) {
				logString("init[FC] > Success\n");
				if (startFlightTasks()) {
					logString("init[FC Start] > Success\n");
					while (1) {
						if (fcStatusData.canFly) {
							statusIndicatorOn();
							vTaskDelay(100);
						} else if (fcStatusData.hasCrashed) {
							statusIndicatorToggle();
							vTaskDelay(50);
						} else if (fcStatusData.canStabilize) {
							statusIndicatorToggle();
							vTaskDelay(100);
						} else if (fcStatusData.canStart) {
							statusIndicatorToggle();
							vTaskDelay(500);
						} else if (!fcStatusData.canStart) {
							statusIndicatorToggle();
							vTaskDelay(1500);
						} else {
							statusIndicatorOff();
							vTaskDelay(100);
						}
					}
				} else {
					logString("init[Flight tasks] > Failure\n");
				}
			} else {
				logString("init[FC] > Failure\n");
			}
		} else {
			logString("init[Indicators] > Failure\n");
		}
	}
}

