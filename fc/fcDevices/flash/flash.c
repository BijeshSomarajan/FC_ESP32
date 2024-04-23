#include "flash.h"
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_err.h"



nvs_handle_t fcNVSFashHandle;
char fcConfigFlashName[] = "fcConfig";

uint8_t initFlash() {
	esp_err_t err = nvs_flash_init();
	return err == ESP_OK;
}

uint8_t writeWordsToFlash(int32_t *data, int32_t length) {
	esp_err_t err = nvs_open(fcConfigFlashName, NVS_READWRITE, &fcNVSFashHandle);
	if (err != ESP_OK) {
		return 0;
	}
	char indxStr[32];
	for (int32_t indx = 0; indx < length; indx++) {
		sprintf(indxStr, "%ld", indx);
		err = nvs_set_i32(fcNVSFashHandle, indxStr, *data);
		if (err != ESP_OK) {
			return 0;
		}
		data++;
	}
	err = nvs_commit(fcNVSFashHandle);
	nvs_close(fcNVSFashHandle);
	return (err == ESP_OK);
}

uint8_t readWordsFromFlash(int32_t *data, int32_t length) {
	esp_err_t err = nvs_open(fcConfigFlashName, NVS_READWRITE, &fcNVSFashHandle);
	if (err != ESP_OK) {
		return 0;
	}
	char indxStr[32];
	for (int32_t indx = 0; indx < length; indx++) {
		sprintf(indxStr, "%ld", indx);
		err = nvs_get_i32(fcNVSFashHandle, indxStr, data);
		if (err != ESP_OK) {
			return 0;
		}
		data++;
	}
	nvs_close(fcNVSFashHandle);
	return (err == ESP_OK);
}

int32_t readWordFromFlash(int32_t indx) {
	esp_err_t err = nvs_open(fcConfigFlashName, NVS_READWRITE, &fcNVSFashHandle);
	if (err != ESP_OK) {
		return 0;
	}
	char indxStr[32];
	sprintf(indxStr, "%ld", indx);
	int32_t data;
	err = nvs_get_i32(fcNVSFashHandle, indxStr, &data);
	if (err != ESP_OK) {
		return 0;
	}
	nvs_close(fcNVSFashHandle);
	return data;
}
