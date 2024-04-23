#include "configSensor.h"

#include "../include/indicatorSensor.h"
#include "fcUART.h"
#include "fcLogger.h"
#include "string.h"

#define  CONFIG_PACKET_DATA_LENGTH 4
#define  CONFIG_PACKET_CMD_REL_INDEX  0
#define  CONFIG_PACKET_DATA_LENGTH_REL_INDEX 1
#define  CONFIG_PACKET_DATA_TUPLE_LENGTH  5
#define  CONFIG_PACKET_DATA_TUPLE_LAST_INDEX  4
#define  CONFIG_PACKET_BUFFER_SIZE 10

#define CONFIG_DATA_BUFFER_SIZE 1024
#define CONFIG_DATA_PACKET_QUEUE_SIZE 5

static uint8_t configProtocolInBuffer[CONFIG_DATA_BUFFER_SIZE];
static int16_t configProtocolInBufferIndex = 0;

uint8_t CONFIG_PROTOCOL_DATA_SEPERATOR = 0x00;
uint8_t CONFIG_PROTOCOL_HEADER[CONFIG_PACKET_DATA_TUPLE_LENGTH] = { 0xfe, 0xff, 0xff, 0xff, 0xff };
uint8_t CONFIG_PROTOCOL_FOOTER[CONFIG_PACKET_DATA_TUPLE_LENGTH] = { 0xff, 0xff, 0xff, 0xff, 0xff };

int16_t configDataPacketCount = 0;
ConfigDataPacket configDataPacketBuffer[CONFIG_PACKET_BUFFER_SIZE];

uint8_t configDataTempReadBuffer[CONFIG_PACKET_DATA_TUPLE_LENGTH];

uint8_t initConfigSensors() {
	if (initUART0()) {
		logString("UART 0 initialized\n");
		if (initUART1()) {
			logString("UART 1 initialized\n");
			return 1;
		} else {
			logString("UART 1 initialization failed\n");
			return 0;
		}
	} else {
		logString("UART 0 initialization failed\n");
		return 0;
	}
}

uint8_t isConfigProtocolHeader(int16_t currentIndex) {
	uint8_t headerMatches = 1;
	int16_t indx = 0;
	int16_t tempCurrentIndex = currentIndex;
	for (indx = CONFIG_PACKET_DATA_TUPLE_LAST_INDEX; tempCurrentIndex >= 0 && indx >= 0; indx--, tempCurrentIndex--) {
		if (CONFIG_PROTOCOL_HEADER[indx] != configProtocolInBuffer[tempCurrentIndex]) {
			headerMatches = 0;
			break;
		}
	}
	if (indx < 0 && headerMatches == 1) {
		return 1;
	} else {
		return 0;
	}
}

uint8_t isConfigProtocolFooter(int16_t currentIndex) {
	uint8_t footerMatches = 1;
	int16_t indx = 0;
	for (indx = CONFIG_PACKET_DATA_TUPLE_LAST_INDEX; currentIndex >= 0 && indx >= 0; indx--, currentIndex--) {
		if (CONFIG_PROTOCOL_FOOTER[indx] != configProtocolInBuffer[currentIndex]) {
			footerMatches = 0;
			break;
		}
	}
	if (indx <= 0 && footerMatches == 1) {
		return 1;
	} else {
		return 0;
	}
}

void handleConfigDataPacket(ConfigDataPacket dataPacket) {
	if (configDataPacketCount < CONFIG_PACKET_BUFFER_SIZE) {
		configDataPacketBuffer[configDataPacketCount] = dataPacket;
		configDataPacketCount++;
	}
}

ConfigDataPacket unmarshalConfigDataPacket(int16_t startIndx, int16_t endIndx) {
	ConfigDataPacket dataPacket;
	int16_t dataIndex = 0;
	int16_t dataValueIndex = 0;
	for (int16_t indx = startIndx; indx <= endIndx; indx += CONFIG_PACKET_DATA_TUPLE_LENGTH) {
		int32_t dataValue = shiftBitsLeft32((0xFF & configProtocolInBuffer[indx + 1]), 24);
		dataValue |= shiftBitsLeft32((0xFF & configProtocolInBuffer[indx + 2]), 16);
		dataValue |= shiftBitsLeft32((0xFF & configProtocolInBuffer[indx + 3]), 8);
		dataValue |= (0xFF & configProtocolInBuffer[indx + 4]);
		if (dataIndex == CONFIG_PACKET_CMD_REL_INDEX) {
			dataPacket.cmd = dataValue;
		} else if (dataIndex == CONFIG_PACKET_DATA_LENGTH_REL_INDEX) {
			dataPacket.length = dataValue;
		} else if (dataValueIndex < CONFIG_DATA_MAX_LENGTH) {
			dataPacket.data[dataValueIndex] = dataValue;
			dataValueIndex++;
		}
		dataIndex++;
	}
	return dataPacket;
}

void updateConfigPacketData(uint8_t *dataBuffer, int16_t len) {
	for (int16_t indx = 0; indx < len; indx++) {
		uint8_t data = dataBuffer[indx];
		configProtocolInBuffer[configProtocolInBufferIndex] = data;
		if (configProtocolInBufferIndex >= CONFIG_PACKET_DATA_TUPLE_LAST_INDEX) {
			if (isConfigProtocolFooter(configProtocolInBufferIndex) == 1) {
				//Adjust for padding before the footer
				configProtocolInBufferIndex -= (CONFIG_PACKET_DATA_TUPLE_LENGTH + 1);
				for (int16_t indx = configProtocolInBufferIndex; indx >= 0; indx -= CONFIG_PACKET_DATA_TUPLE_LENGTH) {
					if (isConfigProtocolHeader(indx)) {
						ConfigDataPacket dataPacket = unmarshalConfigDataPacket(indx + 1, configProtocolInBufferIndex);
						handleConfigDataPacket(dataPacket);
					}
				}
				configProtocolInBufferIndex = 0;
				return;
			}
		}
		configProtocolInBufferIndex++;
		if (configProtocolInBufferIndex >= CONFIG_DATA_BUFFER_SIZE) {
			configProtocolInBufferIndex = 0;
		}
	}
}

int32_t marshalConfigDataPacket(ConfigDataPacket dataPacket, uint8_t *configProtocolOutBuffer) {
	int32_t sendBufferIndx = 0;
	// Adding header
	for (int32_t headerByteIndx = 0; headerByteIndx < CONFIG_PACKET_DATA_TUPLE_LENGTH; headerByteIndx++) {
		configProtocolOutBuffer[sendBufferIndx++] = CONFIG_PROTOCOL_HEADER[headerByteIndx];
	}
	// Adding cmd
	configProtocolOutBuffer[sendBufferIndx++] = CONFIG_PROTOCOL_DATA_SEPERATOR;
	configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) (dataPacket.cmd >> 24);
	configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) (dataPacket.cmd >> 16);
	configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) (dataPacket.cmd >> 8);
	configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) dataPacket.cmd;
	// Adding data length
	configProtocolOutBuffer[sendBufferIndx++] = CONFIG_PROTOCOL_DATA_SEPERATOR;
	configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) (dataPacket.length >> 24);
	configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) (dataPacket.length >> 16);
	configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) (dataPacket.length >> 8);
	configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) dataPacket.length;
	// Adding data
	for (int32_t dataIndx = 0; dataIndx < dataPacket.length; dataIndx++) {
		configProtocolOutBuffer[sendBufferIndx++] = CONFIG_PROTOCOL_DATA_SEPERATOR;
		configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) (dataPacket.data[dataIndx] >> 24);
		configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) (dataPacket.data[dataIndx] >> 16);
		configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) (dataPacket.data[dataIndx] >> 8);
		configProtocolOutBuffer[sendBufferIndx++] = (uint8_t) dataPacket.data[dataIndx];
	}
	// Adding footer
	configProtocolOutBuffer[sendBufferIndx++] = CONFIG_PROTOCOL_DATA_SEPERATOR;
	for (int32_t footerByteIndx = 0; footerByteIndx < CONFIG_PACKET_DATA_TUPLE_LENGTH; footerByteIndx++) {
		configProtocolOutBuffer[sendBufferIndx++] = CONFIG_PROTOCOL_FOOTER[footerByteIndx];
	}
	return sendBufferIndx;
}

uint8_t hasConfigDataPacket() {
	if (configDataPacketCount > 0) {
		return 1;
	} else {
		return 0;
	}
}

ConfigDataPacket getConfigDataPacket() {
	ConfigDataPacket dataPacket = configDataPacketBuffer[0];
	if (configDataPacketCount > 0) {
		for (int32_t indx = 0; indx < configDataPacketCount - 1; indx++) {
			configDataPacketBuffer[indx] = configDataPacketBuffer[indx + 1];
		}
		configDataPacketCount--;
	}
	return dataPacket;
}

void readConfigSensors() {
	int bytesRead = readBytesFromUART0(configDataTempReadBuffer, CONFIG_PACKET_DATA_TUPLE_LENGTH);
	if (bytesRead > 0) {
		updateConfigPacketData(configDataTempReadBuffer, bytesRead);
	}
}

void sendConfigDataPacket(ConfigDataPacket dataPacket) {
	uint8_t configProtocolOutBuffer[CONFIG_DATA_BUFFER_SIZE];
	int32_t bufferSize = marshalConfigDataPacket(dataPacket, configProtocolOutBuffer);
	writeBytesToUART0(configProtocolOutBuffer, bufferSize + 1);
}

void sendConfigData(int32_t *data, int32_t length, int32_t cmd) {
	if (length <= CONFIG_DATA_MAX_LENGTH) {
		ConfigDataPacket dataPacket;
		dataPacket.cmd = cmd;
		dataPacket.length = length;
		for (int32_t indx = 0; indx < length; indx++) {
			dataPacket.data[indx] = data[indx];
		}
		sendConfigDataPacket(dataPacket);
	}
}


