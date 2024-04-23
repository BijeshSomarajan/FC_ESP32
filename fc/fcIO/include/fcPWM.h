#ifndef FC_FCIO_INCLUDE_FCPWM_H_
#define FC_FCIO_INCLUDE_FCPWM_H_
#include <stdio.h>
#include <inttypes.h>

#define PWM_CHANNEL_COUNT  4

uint8_t initPWM(void);
uint8_t startPWMs(void);
uint8_t stopPWMs(void);
void setPWMValue(uint8_t channel, uint16_t pwmValue);

#endif
