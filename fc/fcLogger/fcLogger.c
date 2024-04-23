#include "fcLogger.h"
#include "fcUART.h"

uint8_t initLogger() {
	uint8_t status = initUART0();
	if (status) {
		logString("Logger initialized on UART-0\n");
	}
	return status;
}

void logString(char *data) {
	writeBytesToUART0((uint8_t*) data, strlen(data));
}

