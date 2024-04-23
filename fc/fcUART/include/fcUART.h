#ifndef FC_FCIO_INCLUDE_FCUART_H_
#define FC_FCIO_INCLUDE_FCUART_H_
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

uint8_t initUART0(void);
uint8_t initUART1(void);
uint8_t initUART2(void);

int16_t readBytesFromUART0(uint8_t *data, int16_t len);
uint8_t writeBytesToUART0(uint8_t *data, int16_t len);

int16_t readBytesFromUART1(uint8_t *data, int16_t len);
uint8_t writeBytesToUART1(uint8_t *data, int16_t len);

int16_t readBytesFromUART2(uint8_t *data, int16_t len);
uint8_t writeBytesToUART2(uint8_t *data, int16_t len);

#endif
