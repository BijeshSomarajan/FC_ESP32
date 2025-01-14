#ifndef FCDEBUGGER_H_
#define FCDEBUGGER_H_
#include <inttypes.h>

#define FC_DEBUG_ENABLED 0
#define FC_DEBUG_PERIOD 1.0f/50.0f
#define FC_DEBUG_PERIOD_US FC_DEBUG_PERIOD * 1000000.0f

uint8_t initDebugManager(void);
uint8_t startDebugManager(void);

#endif
