#ifndef _PWMSENSOR_H_
#define _PWMSENSOR_H_
#include <stdio.h>
#include <inttypes.h>
#include <fcPWM.h>

uint8_t initPWMSensors(void);
uint8_t startPWMSensors(void);
uint8_t stopPWMSensors(void);
void setPWMChannelValue(uint8_t channel, int value);
void updatePWMValues(void);

typedef struct _PWM_DATA PWM_DATA;
struct _PWM_DATA {
	int PWM_VALUES[PWM_CHANNEL_COUNT];
	uint8_t cpu;
	float dt;
};
extern PWM_DATA pwmData;

#endif
