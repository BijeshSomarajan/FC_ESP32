#include "../../devices/bmp280/bmp280Helper.h"

//Note: Size of buffer is optimized as per the need
uint8_t bmp280WriteBuffer[25];
/*****************************************************************************/
/* Reads the specified number of bytes  from the device register              */
/*****************************************************************************/
int bmp280ReadRegister(uint8_t regAddr,uint8_t* data,int length){
	data[0] = regAddr;
	if (spi6Read1(data, length) == 1) {
    	return length;
    }else{
    	return 0;
    }
}

/*****************************************************************************/
/* Writes the specified number of bytes  to the device register              */
/*****************************************************************************/
int bmp280WriteRegister(uint8_t regAddr,uint8_t* data,int length){
	int32_t len = 0;
	if (length > 0) {
		bmp280WriteBuffer[0] = regAddr;
		memcpy(bmp280WriteBuffer + 1, data, length);
		if (spi6Write1(bmp280WriteBuffer, length + 1) == 1) {
			len = 1;
		}
	}
	return len;
}
