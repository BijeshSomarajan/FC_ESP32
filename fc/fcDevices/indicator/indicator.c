#include "indicator.h"
#include "driver/gpio.h"

#define STATUS_OUTPUT_IO  2

uint8_t initIndicators() {
	esp_err_t err = gpio_set_direction(STATUS_OUTPUT_IO, GPIO_MODE_INPUT_OUTPUT);
	return err == ESP_OK;
}

void statusIndicatorOn() {
	gpio_set_level(STATUS_OUTPUT_IO, 1);
}

void statusIndicatorOff() {
	gpio_set_level(STATUS_OUTPUT_IO, 0);
}

void statusIndicatorToggle() {
	int statusState = gpio_get_level(STATUS_OUTPUT_IO);
	if (statusState == 1) {
		gpio_set_level(STATUS_OUTPUT_IO, 0);
	} else {
		gpio_set_level(STATUS_OUTPUT_IO, 1);
	}
}

