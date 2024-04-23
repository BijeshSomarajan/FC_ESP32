#include "fsia.h"
#include "fcLogger.h"

#define FSIA_BUFFSIZE 32
#define FSIA_CKSUM_LSB_INDX FSIA_BUFFSIZE-2
#define FSIA_CKSUM_HSB_INDX FSIA_BUFFSIZE-1

uint16_t fsiaChannelValue[FSIA_CHANNEL_COUNT] = { 0 };
uint8_t fsiaIndex = 0;
uint8_t fsiaData[FSIA_BUFFSIZE] = { 0 };
uint8_t fsiaTempData[FSIA_BUFFSIZE] = { 0 };
uint16_t fsiaMessageCount = 0;
uint16_t fsiaInitState = 0;
/**
 * Initializes the iBUS protocol
 **/
uint8_t initFSIA() {
	if (initUART2()) {
		fsiaInitState = 1;
		logString("[fsai,IO:UART-2] > Success\n");
		return 1;
	} else {
		logString("[fsai,IO:UART-2] > Failure\n");
		return 0;
	}
}

/**
 * Gets the value of RC Channel
 **/
uint16_t getFSIAChannelValue(uint8_t channel) {
	return fsiaChannelValue[channel];
}

void resetFSIAState() {
	fsiaMessageCount = 0;
}

uint8_t isFSIAActiveState() {
	if (fsiaMessageCount == 0) {
		return 0;
	} else {
		return 1;
	}
}

/**
 * Callback method for iBus DataByte
 * Frame example:
 * x20x40   = Header (length of 32 bytes + command)
 * xDCx05  = 1500 ch 1
 * xDBx05  = 1499 ch 2
 * xEFx03  = 1007 ch 3
 * xDDx05  = 1501 ch 4
 * xD0x07  = 2000 ch 5
 * xD0x07  = 2000 ch 6
 * xDCx05  = 1500 ch 7
 * xDCx05  = 1500 ch 8
 * xDCx05  = 1500 ch 9
 * xDCx05  = 1500 ch 10
 * xDCx05  = 1500 ch 11
 * xDCx05  = 1500 ch 12
 * xDCx05  = 1500 ch 13
 * xDCx05  = 1500 ch 14
 * x54xF3  = Checksum:  0xFFFF - (0x20 + 0x40 ... sum of all !)
 **/
uint8_t fsiaPrevDataByte = 0;
uint16_t fsiaChksum = 0xFF9F; //0xFFFF-0x40-0x20
void updateFSIAData(uint8_t *dataBytes, uint16_t length) {
	for (uint16_t indx = 0; indx < length; indx++) {
		uint8_t data = dataBytes[indx];
		if (data == 0x40 && fsiaPrevDataByte == 0x20) {
			fsiaData[0] = 0x20;
			fsiaData[1] = 0x40;
			fsiaChksum = 0xFF9F;
			fsiaIndex = 2;
		} else if (fsiaIndex > 1 && fsiaIndex < FSIA_BUFFSIZE) {
			fsiaData[fsiaIndex] = data;
			if (fsiaIndex == FSIA_CKSUM_HSB_INDX) {
				uint16_t rxChksum = fsiaData[FSIA_CKSUM_LSB_INDX] + (fsiaData[FSIA_CKSUM_HSB_INDX] << 8);
				if (fsiaChksum == rxChksum) {
					fsiaChannelValue[0] = (fsiaData[3] << 8) + fsiaData[2];
					fsiaChannelValue[1] = (fsiaData[5] << 8) + fsiaData[4];
					fsiaChannelValue[2] = (fsiaData[7] << 8) + fsiaData[6];
					fsiaChannelValue[3] = (fsiaData[9] << 8) + fsiaData[8];
					fsiaChannelValue[4] = (fsiaData[11] << 8) + fsiaData[10];
					fsiaChannelValue[5] = (fsiaData[13] << 8) + fsiaData[12];
					fsiaChannelValue[6] = (fsiaData[15] << 8) + fsiaData[14];
					fsiaChannelValue[7] = (fsiaData[17] << 8) + fsiaData[16];
					fsiaChannelValue[8] = (fsiaData[19] << 8) + fsiaData[18];
					fsiaChannelValue[9] = (fsiaData[21] << 8) + fsiaData[20];
					if (fsiaMessageCount == 0 || fsiaMessageCount > FSIA_MAX_MESSAGE_COUNT) {
						fsiaMessageCount = 1;
					}
					fsiaMessageCount++;
				}
				fsiaIndex = 0;
			} else {
				if (fsiaIndex < FSIA_CKSUM_LSB_INDX) {
					fsiaChksum -= fsiaData[fsiaIndex];
				}
				fsiaIndex++;
			}
		} else {
			fsiaIndex = 0;
		}
		//Store for checking header start
		fsiaPrevDataByte = data;
	}
}

void readFSIA(void) {
	if (fsiaInitState == 1) {
		int16_t bytesRead = readBytesFromUART2(fsiaTempData, FSIA_BUFFSIZE);
		if (bytesRead > 0) {
			updateFSIAData(fsiaTempData, bytesRead);
		}
	}
}
