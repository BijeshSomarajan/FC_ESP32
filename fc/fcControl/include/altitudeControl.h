#ifndef _ALTITUDECONTROL_H_
#define _ALTITUDECONTROL_H_

#include <stdio.h>
#include <inttypes.h>

uint8_t initAltitudeControl();
void resetAltitudeControl(uint8_t hard);
void controlAltitudeWithGain(float dt, float altitude, float altitudeRef, float kPGain, float kDGain, float rateKPGain, float rateKDGain);

#endif
