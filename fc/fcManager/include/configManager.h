#ifndef CONFIGURATIONMANAGER_H_
#define CONFIGURATIONMANAGER_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#define CONFIG_DATA_READ_PERIOD 1.0/2000.0f
#define CONFIG_DATA_PROCESS_PERIOD 1.0/500.0f

uint8_t initConfigManager(void);
uint8_t startConfigManager(void);

uint8_t manageConfigurationSave(void);
void manageConfigDataPacket(void);
uint8_t hasNewConfiguration(void);

#endif
