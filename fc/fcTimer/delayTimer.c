#include "delayTimer.h"

/**
 * Delays for the given amount of milliseconds
 */
void delayMs(uint32_t delay) {
	vTaskDelay(pdMS_TO_TICKS(delay));
}
/**
 * Delays for the given amount of microseconds
 */
void delayUs(uint32_t delay) {
	ets_delay_us(delay * 1000);
}

/**
 * Initializes the delay timer
 */
uint8_t initDelayTimer() {
	return 1;
}
