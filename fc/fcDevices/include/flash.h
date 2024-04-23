#ifndef STORAGE_FLASH_H_
#define STORAGE_FLASH_H_
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

uint8_t initFlash(void);
uint8_t writeWordsToFlash(int32_t* data, int32_t length);
uint8_t readWordsFromFlash(int32_t *data, int32_t length);
int32_t readWordFromFlash(int32_t indx);

#endif /* STORAGE_FLASH_H_ */
