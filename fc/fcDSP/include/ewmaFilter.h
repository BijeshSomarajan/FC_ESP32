#ifndef FC_FCDSP_INCLUDE_EWMAFILTER_H_
#define FC_FCDSP_INCLUDE_EWMAFILTER_H_
#include <math.h>
#include "mathUtil.h"

typedef struct _EWMAFILTER EWMAFILTER;
struct _EWMAFILTER{
    float mean;
    float lamda;
    float output;
} ;

// Initialize the EWMA structure
void initEWMAFilter(EWMAFILTER *ewma,float lamda);

// Function to update EWMA mean and remove bias
float updateEWMAFilter(EWMAFILTER *ewma, float value);

void resetEWMAFilter(EWMAFILTER *ewma);

#endif /* FC_FCDSP_INCLUDE_EWMAFILTER_H_ */
