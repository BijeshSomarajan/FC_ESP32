#ifndef _INDICATOR_H_
#define _INDICATOR_H_

#include <stdio.h>
#include <inttypes.h>

uint8_t initIndicators(void);
void statusIndicatorOn(void);
void statusIndicatorOff(void);
void statusIndicatorToggle(void);

#endif
