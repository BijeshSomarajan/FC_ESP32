#ifndef RC_PROTOCOL_FSIA_H_
#define RC_PROTOCOL_FSIA_H_

#include <math.h>
#include <stdint.h>
#include <inttypes.h>
#include "fcUART.h"

#define FSIA_CHANNEL_COUNT 10
#define FSIA_CHANNEL_MIN_VALUE 1000
#define FSIA_CHANNEL_MAX_VALUE 2000
#define FSIA_CHANNEL_MID_VALUE 1500
#define FSIA_MAX_MESSAGE_COUNT 500

uint8_t initFSIA(void);
uint16_t getFSIAChannelValue(uint8_t channel);
void readFSIA(void);
void resetFSIAState(void) ;
uint8_t isFSIAActiveState(void);

#endif
