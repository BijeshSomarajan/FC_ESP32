#ifndef MODULES_ATTITUDE_CONTROL_H_
#define MODULES_ATTITUDE_CONTROL_H_
#include <stdio.h>
#include <inttypes.h>

uint8_t initAttitudeControl();
void resetAttitudeControl(uint8_t hard);
void controlAttitude(float dt, float expectedPitch, float expectedRoll, float expectedYaw, float pitchRollPGain, float pitchRollIGain, float pitchRollRatePGain, float  pitchRollRateDGain);

#define ATT_CONTROL_D_RATE_LPF_FREQ 100.0f

#define ATT_CONTROL_MASTER_DB_P_ENABLED 0
#define ATT_CONTROL_MASTER_DB_P 0.5f

#define ATT_CONTROL_MASTER_DB_R_ENABLED 0
#define ATT_CONTROL_MASTER_DB_R 0.5f

#endif
