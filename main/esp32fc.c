#include <stdio.h>
#include <inttypes.h>

#include "fcManager.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "soc/rtc.h"
#include "hal/spi_types.h"
#include "sdkconfig.h"


void app_main(void) {
	printf("Starting : ESP32\n");
	startFlightController();
}
