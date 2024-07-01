#include "fcWifiIO.h"

#define WIFI_SERVER_PORT 8001
#define WIFI_MAX_RCV_SIZE 128

uint8_t wifiRcvBuffer[WIFI_MAX_RCV_SIZE];
uint8_t wifiRespBuffer[WIFI_MAX_RCV_SIZE];

TaskHandle_t wifiUDPServerTaskHandle;
uint8_t wifiRequestUnderProccess = 0;

extern void handleWifiRxData(uint8_t *inputData, int inputLen, uint8_t *outputData, uint16_t *outputLen);

void wifiUDPServerTask(void *pvParameters) {
	int ap_sockfd = *(int*) pvParameters;
	struct sockaddr_in source_addr;
	int sockaddr_len = sizeof(struct sockaddr_in);
	in_port_t sin_port = htons(WIFI_SERVER_PORT);
	while (1) {
		ssize_t size = recvfrom(ap_sockfd, wifiRcvBuffer, WIFI_MAX_RCV_SIZE, 0, (struct sockaddr*) &source_addr, (socklen_t*) &sockaddr_len);
		if (size > 0) {
			uint16_t respSize = 0;
			handleWifiRxData(wifiRcvBuffer, size,wifiRespBuffer,&respSize);
			if (respSize > 0) {
				source_addr.sin_port = sin_port;
				sendto(ap_sockfd, wifiRespBuffer, respSize, 0, (struct sockaddr*) &source_addr, sockaddr_len);
			}
		}
	}
	vTaskDelete(NULL);
}

uint8_t startUDPServer() {
	int ap_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
	if (ap_sockfd < 0) {
		logString("WIFI : startUDPServer , Socket creation Failed\n");
		return 0;
	}

	struct timeval timeout;
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	setsockopt(ap_sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

	struct sockaddr_in self_ap_addr;
	self_ap_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	self_ap_addr.sin_family = AF_INET;
	self_ap_addr.sin_port = htons(WIFI_SERVER_PORT);

	if (bind(ap_sockfd, (const struct sockaddr*) &self_ap_addr, sizeof(struct sockaddr_in)) < 0) {
		shutdown(ap_sockfd, 0);
		close(ap_sockfd);
		logString("WIFI : startUDPServer , Socket binding Failed\n");
		return 0;
	}

	logString("WIFI : startUDPServer , Socket Bound\n");

	BaseType_t err = xTaskCreatePinnedToCore(wifiUDPServerTask, "wifiUDPServerTask", 1024 * 4, &ap_sockfd, 10, &wifiUDPServerTaskHandle, xPortGetCoreID());
	logString("WIFI : startUDPServer , Server Started\n");

	return (err == pdPASS);
}

