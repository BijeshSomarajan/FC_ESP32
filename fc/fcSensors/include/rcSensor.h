#ifndef FC_SENSOR_RC_RCSENSOR_H_
#define FC_SENSOR_RC_RCSENSOR_H_
#include <stdio.h>
#include <inttypes.h>
#include "fsia.h"

typedef struct _RC_DATA RC_DATA;
struct _RC_DATA {
	//RC stick middle values
	int16_t RC_MID_DATA[10];
	//RC Delta values
	int16_t RC_DELTA_DATA[10];
	//RC effective values
	int16_t RC_EFFECTIVE_DATA[10];
	//Flags to state the stick positions
	uint8_t pitchCentered, rollCentered, yawCentered, throttleCentered ;
	uint8_t cpu;
	float readDt;
	float processDt;
};
extern RC_DATA rcData;

//Number of RC channels
#define RC_CHANNEL_COUNT 10
#define RC_CHANNEL_MAX_INDEX (RC_CHANNEL_COUNT-1)
#define RC_CHANNEL_DEAD_BAND 3

//Channel Indexes , M3
#define RC_YAW_CHANNEL_INDEX          0 // Channel 1
#define RC_TH_CHANNEL_INDEX           1 // Channel 2
#define RC_PITCH_CHANNEL_INDEX        2 // Channel 4
#define RC_ROLL_CHANNEL_INDEX         3 // Channel 3

#define RC_FLIGHT_MODE_CHANNEL_INDEX  4 //Channel 5 //Reserved
#define RC_AUX_CHANNEL5_INDEX         5 //Channel 6 //None

#define RC_START_CHANNEL_INDEX       6 //Channel 7 //swa
#define RC_POS_CHANNEL_INDEX         7 //Channel 8  //swb
#define RC_HEADING_CHANNEL_INDEX     8 //Channel 9  //swc
#define RC_LAND_CHANNEL_INDEX        9 //Channel 10 //Land

#define RC_CHANNEL_MIN_VALUE FSIA_CHANNEL_MIN_VALUE
#define RC_CHANNEL_MAX_VALUE FSIA_CHANNEL_MAX_VALUE
#define RC_CHANNEL_MID_VALUE FSIA_CHANNEL_MID_VALUE
#define RC_CHANNEL_DELTA_VALUE (RC_CHANNEL_MAX_VALUE-RC_CHANNEL_MIN_VALUE)

uint8_t initRCSensors(void);
void readRCSensors(void);

uint16_t getRCValue(uint8_t channel);
void calibrateRCSensor(void);

#endif /* FC_SENSOR_RC_RCSENSOR_H_ */
