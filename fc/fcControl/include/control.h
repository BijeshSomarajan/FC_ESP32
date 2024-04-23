#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <stdio.h>
#include <inttypes.h>

typedef struct _CONTROL_DATA CONTROL_DATA;
struct _CONTROL_DATA {
	float throttleControl;
	float pitchControl;
	float rollControl;
	float yawControl;
	float altitudeControl;
	float tiltCompensationThrottle;
	float posHoldCompensationThrottle;
	float positionPitchControl, positionRollControl;
	float attitudeControlDt;
	float altitudeControlDt;
};

extern CONTROL_DATA controlData;

#endif
