#include "fcUART.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"

#define UART0_BUFFER_SIZE 1024
#define UART0_EVENT_QUEUE_SIZE 10

#define UART1_TX_PIN 10 //SD3
#define UART1_RX_PIN 9  //SD2
#define UART1_BUFFER_SIZE 1024
#define UART1_EVENT_QUEUE_SIZE 10

#define UART2_TX_PIN 17
#define UART2_RX_PIN 16
#define UART2_BUFFER_SIZE 1024
#define UART2_EVENT_QUEUE_SIZE 10

QueueHandle_t uart0Queue;
QueueHandle_t uart1Queue;
QueueHandle_t uart2Queue;

uint8_t UART0InitStatus = 0;
uint8_t UART1InitStatus = 0;
uint8_t UART2InitStatus = 0;

const uart_config_t uart0Config = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, // 0
		.stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };

const uart_config_t uart1Config = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, // 0
		.stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };

const uart_config_t uart2Config = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, // 0
		.stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };

uint8_t initUART0() {
	if (!UART0InitStatus) {
		uart_param_config(UART_NUM_0, &uart0Config);
		uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
		esp_err_t err = uart_driver_install(UART_NUM_0, UART0_BUFFER_SIZE, UART0_BUFFER_SIZE, UART0_EVENT_QUEUE_SIZE, &uart0Queue, 0);
		if (err != ESP_OK) {
			return 0;
		} else {
			UART0InitStatus = 1;
			return 1;
		}
	} else {
		return 1;
	}
}

uint8_t initUART1() {
	if (!UART1InitStatus) {
		uart_param_config(UART_NUM_1, &uart1Config);
		uart_set_pin(UART_NUM_1, UART1_TX_PIN, UART1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
		esp_err_t err = uart_driver_install(UART_NUM_1, UART1_BUFFER_SIZE, UART1_BUFFER_SIZE, UART1_EVENT_QUEUE_SIZE, &uart1Queue, 0);
		if (err != ESP_OK) {
			return 0;
		} else {
			UART1InitStatus = 1;
			return 1;
		}
	} else {
		return 1;
	}
}

uint8_t initUART2() {
	if (!UART2InitStatus) {
		uart_param_config(UART_NUM_2, &uart2Config);
		uart_set_pin(UART_NUM_2, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
		esp_err_t err = uart_driver_install(UART_NUM_2, UART2_BUFFER_SIZE, UART2_BUFFER_SIZE, UART2_EVENT_QUEUE_SIZE, &uart2Queue, 0);
		if (err != ESP_OK) {
			return 0;
		} else {
			UART2InitStatus = 1;
			return 1;
		}
	}
	return 1;
}

int16_t readBytesFromUART0(uint8_t *data, int16_t len) {
	return uart_read_bytes(UART_NUM_0, data, len, 0);
}

uint8_t writeBytesToUART0(uint8_t *data, int16_t len) {
	int txLen = uart_write_bytes(UART_NUM_0, data, len);
	return txLen == len;
}

int16_t readBytesFromUART1(uint8_t *data, int16_t len) {
	return uart_read_bytes(UART_NUM_1, data, len, 0);
}

uint8_t writeBytesToUART1(uint8_t *data, int16_t len) {
	int txLen = uart_write_bytes(UART_NUM_1, data, len);
	return txLen == len;
}

int16_t readBytesFromUART2(uint8_t *data, int16_t len) {
	return uart_read_bytes(UART_NUM_2, data, len, 0);
}

uint8_t writeBytesToUART2(uint8_t *data, int16_t len) {
	int txLen = uart_write_bytes(UART_NUM_2, data, len);
	return txLen == len;
}
