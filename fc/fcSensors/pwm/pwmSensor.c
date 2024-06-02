#include "pwmSensor.h"
#include "fcLogger.h"
#include "fcPwm.h"

PWM_DATA pwmData;

uint8_t initPWMSensors(void) {
	if (initPWM()) {
		logString("[pwm] Init - Success\n");
	} else {
		logString("[pwm] Init - Failed\n");
		return 0;
	}
	return 1;
}

uint8_t startPWMSensors() {
	if (startPWMs()) {
		logString("[pwm] Start - Success\n");
	} else {
		logString("[pwm] Start - Failed\n");
		return 0;
	}
	return 1;
}
uint8_t stoptPWMSensors() {
	if (stopPWMs()) {
		logString("[pwm] Stop - Success\n");
	} else {
		logString("[pwm] Stop - Failed\n");
		return 0;
	}
	return 1;
}

void setPWMChannelValue(uint8_t channel, int value) {
	pwmData.PWM_VALUES[channel] = value;
	setPWMValue(channel, value);
}

void updatePWMValues() {
	for (uint8_t cCount = 0; cCount < PWM_CHANNEL_COUNT; cCount++) {
		setPWMValue(cCount, pwmData.PWM_VALUES[cCount] < 0 ? 0 : pwmData.PWM_VALUES[cCount]);
	}
}

