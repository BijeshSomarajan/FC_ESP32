#include "deltaTimer.h"

int64_t fcLastTime = 0;
/**
 * Gets the elapsed time between consecutive calls
 */
float getDeltaTime() {
	int64_t timeNow = esp_timer_get_time();
	float deltaUs = (float) (timeNow - fcLastTime) / 1000000.0f;
	fcLastTime = timeNow;
	return deltaUs;
}

int64_t getTimeUSec() {
	return esp_timer_get_time();
}

float getUSecTimeInSec(int64_t usecTime){
	return (float)usecTime/1000000.0f;
}



/**
 * Resets the accumulated elapse time
 */
void resetDeltaTime() {
}

/**
 * Initializes the delta timer
 */
uint8_t initDeltaTimer() {
	fcLastTime = esp_timer_get_time();
	return 1;
}
