#ifndef FC_IO_SPI_SPI_H_
#define FC_IO_SPI_SPI_H_

#include <stdio.h>
#include <inttypes.h>


#define SPI3_DEVICE_1 1
#define SPI3_DEVICE_2 2
#define SPI3_DEVICE_3 3

uint8_t initSPI3(void);
uint8_t spi3Write(uint8_t addr, uint8_t *txData, uint8_t length, uint8_t devId);
uint8_t spi3Read(uint8_t addr, uint8_t *rxData, uint8_t length, uint8_t devId);

#endif
