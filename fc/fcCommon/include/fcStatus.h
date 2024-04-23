#ifndef FC_STATUS_H_
#define FC_STATUS_H_

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

typedef struct _FC_STATUS_DATA FC_STATUS_DATA;
struct _FC_STATUS_DATA {
	//Enabled states
	uint8_t enableAltitudeHold, enableGlobalPositionHold, enableTerrainPosHold, enableRTH, enableTerrainAlt;
	//Enabled status
	uint8_t altitudeHoldEnabled;
	//Active states
	uint8_t canStart, canArm, isArmed, isJustArmed, hasCrashLanded, isGlobalPosHoldModeActive, isRTHModeActive, isTerrainAltModeActive, isHeadLessModeActive;
	//Flag to state if idle or not
	uint8_t hasTakenOff;
	//Flag to state if landing landing mode is active
	uint8_t isLandingModeActive, isLandingModeActiveAfterRTH, isFailSafeLandingMode;
	//Flight debug status enabled
	uint8_t debugEnabled;
	//The position references
	double positionXRef, positionYRef;
	//The home position
	double positionXHome, positionYHome;
	uint8_t isPositionHomeSet;

	//Flight reference values
	float headingRef, headingHomeRef;
	float altitudeRefSeaLevel;

	//Throttle reference values
	float currentThrottle;
	float throttlePercentage;

	//The config mode
	uint8_t isConfigMode;
};

extern FC_STATUS_DATA fcStatusData;

#endif
