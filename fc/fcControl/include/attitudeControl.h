#ifndef MODULES_ATTITUDE_CONTROL_H_
#define MODULES_ATTITUDE_CONTROL_H_
#include <stdio.h>
#include <inttypes.h>

uint8_t initAttitudeControl();
void resetAttitudeControl(uint8_t hard);
void controlAttitude(float dt, float expectedPitch, float expectedRoll, float expectedYaw, float pitchRollPGain, float pitchRollIGain, float pitchRollRatePGain, float  pitchRollRateDGain);

#define ATT_CONTROL_D_RATE_LPF_FREQ 0.0f



#endif
