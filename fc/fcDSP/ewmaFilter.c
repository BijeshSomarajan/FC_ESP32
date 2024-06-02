#include "ewmaFilter.h"

void initEWMAFilter(EWMAFILTER *ewma,float lamda) {
    ewma->mean = 0.0f;
    ewma->output = 0.0f;
    ewma->lamda = lamda;
}

// Function to update EWMA mean and remove bias
float updateEWMAFilter(EWMAFILTER *ewma, float value) {
    ewma->mean = ewma->lamda * value + (1.0f - ewma->lamda) * ewma->mean;
    ewma->output = value - ewma->mean;
    return ewma->output;
}

void resetEWMAFilter(EWMAFILTER *ewma){
    ewma->mean = 0.0f;
    ewma->output = 0.0f;
}



