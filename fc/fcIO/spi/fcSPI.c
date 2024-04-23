#include "fcSPI.h"
#include "fcLogger.h"
#include "hal/spi_types.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "esp_system.h"
#include "esp_err.h"

SemaphoreHandle_t SPI3Mutex;
#define SPI3_MUTEX_TIME_OUT 0

#define SPIBUS_READ_MASK     (0x80)  /*!< addr | SPIBUS_READ_MASK  */
#define SPIBUS_WRITE_MASK    (0x7F)  /*!< addr & SPIBUS_WRITE_MASK */

#define SPI3_PIN_MISO 19
#define SPI3_PIN_MOSI 23
#define SPI3_PIN_CLK  18
#define SPI3_PIN_CS_DEV_1 5
#define SPI3_PIN_CS_DEV_2 4
#define SPI3_PIN_CS_DEV_3 15
#define SPI3_CLK_SPEED 10000000 //10Mz
#define SPI3_QUEUE_SIZE 10

uint8_t SPI3_INIT_STATUS = 0;

spi_bus_config_t spi3BusConfig = {
		.miso_io_num = SPI3_PIN_MISO,
		.mosi_io_num = SPI3_PIN_MOSI,
		.sclk_io_num = SPI3_PIN_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 128 };

spi_device_interface_config_t spi3Dev1Config = {
		.clock_speed_hz = SPI3_CLK_SPEED,        //Clock out at 1 MHz
		.mode = 0,                         //SPI mode 0
		.spics_io_num = SPI3_PIN_CS_DEV_1,  //CS pin
		.queue_size = SPI3_QUEUE_SIZE,
		.address_bits = 8,
		.input_delay_ns = 0,
		};

spi_device_interface_config_t spi3Dev2Config = {
		.clock_speed_hz = SPI3_CLK_SPEED,        //Clock out at 1 MHz
		.mode = 0,                         //SPI mode 0
		.spics_io_num = SPI3_PIN_CS_DEV_2,  //CS pin
		.queue_size = SPI3_QUEUE_SIZE,
		.address_bits = 8,
		.input_delay_ns = 0,
		};

spi_device_interface_config_t spi3Dev3Config = {
		.clock_speed_hz = SPI3_CLK_SPEED,        //Clock out at 1 MHz
		.mode = 0,                         //SPI mode 0
		.spics_io_num = SPI3_PIN_CS_DEV_3,  //CS pin
		.queue_size = SPI3_QUEUE_SIZE,
		.address_bits = 8,
		.input_delay_ns = 0,
		};

spi_device_handle_t spi3Device1Handle;
spi_device_handle_t spi3Device2Handle;
spi_device_handle_t spi3Device3Handle;

spi_transaction_t spi3DeviceTransactionHandle = {
		.flags = 0,
		.cmd = 0,
		.addr = 0,
		.length = 0,
		.rxlength = 0,
		.user = 0,
		.tx_buffer = 0,
		.rx_buffer = 0
};

uint8_t initSPI3() {
	uint8_t status = SPI3_INIT_STATUS;
	if (!SPI3_INIT_STATUS) {
		SPI3Mutex = xSemaphoreCreateMutex();
		logString("SPI-3 , Mutex Created\n");

		esp_err_t err = spi_bus_initialize(SPI3_HOST, &spi3BusConfig, SPI_DMA_CH_AUTO); //);
		if (err == ESP_OK) {
			logString("SPI-3 , Bus Initialized\n");
			status = 1;
		} else {
			logString("SPI-3 , Bus Initialization Failed!\n");
			return 0;
		}

		err = spi_bus_add_device(SPI3_HOST, &spi3Dev1Config, &spi3Device1Handle);
		if (err == ESP_OK) {
			logString("SPI-3 , Device-1 Added\n");
			status = 1;
		} else {
			logString("SPI-3 , Device-1 Addition Failed!\n");
			return 0;
		}

		err = spi_bus_add_device(SPI3_HOST, &spi3Dev2Config, &spi3Device2Handle);
		if (err == ESP_OK) {
			logString("SPI-3 , Device-2 Added\n");
			status = 1;
		} else {
			logString("SPI-3 , Device-2 Addition Failed!\n");
			return 0;
		}

		err = spi_bus_add_device(SPI3_HOST, &spi3Dev3Config, &spi3Device3Handle);
		if (err == ESP_OK) {
			logString("SPI-3 , Device-3 Added\n");
			status = 1;
		} else {
			logString("SPI-3 , Device-3 Addition Failed!\n");
			return 0;
		}

		SPI3_INIT_STATUS = status;
	}
	return status;
}

uint8_t spi3Write(uint8_t addr, uint8_t *txData, uint8_t length, uint8_t devId) {
	if (xSemaphoreTake(SPI3Mutex, SPI3_MUTEX_TIME_OUT) == pdTRUE) {
		spi3DeviceTransactionHandle.tx_buffer = txData;
		spi3DeviceTransactionHandle.rx_buffer = 0;
		spi3DeviceTransactionHandle.length = length * 8;
		spi3DeviceTransactionHandle.rxlength = 0;
		spi3DeviceTransactionHandle.addr = addr & SPIBUS_WRITE_MASK;
		spi_device_handle_t selectedHandle;
		if (devId == SPI3_DEVICE_1) {
			selectedHandle = spi3Device1Handle;
		} else if (devId == SPI3_DEVICE_2) {
			selectedHandle = spi3Device2Handle;
		} else {
			selectedHandle = spi3Device3Handle;
		}
		esp_err_t err = spi_device_transmit(selectedHandle, &spi3DeviceTransactionHandle);
		xSemaphoreGive(SPI3Mutex);
		return err == ESP_OK;
	} else {
		return 0;
	}
}

uint8_t spi3Read(uint8_t addr, uint8_t *rxData, uint8_t length, uint8_t devId) {
	if (xSemaphoreTake(SPI3Mutex, SPI3_MUTEX_TIME_OUT) == pdTRUE) {
		spi3DeviceTransactionHandle.tx_buffer = 0;
		spi3DeviceTransactionHandle.rx_buffer = rxData;
		spi3DeviceTransactionHandle.length = length * 8;
		spi3DeviceTransactionHandle.rxlength = length * 8;
		spi3DeviceTransactionHandle.addr = addr | SPIBUS_READ_MASK;
		spi_device_handle_t selectedHandle;
		if (devId == SPI3_DEVICE_1) {
			selectedHandle = spi3Device1Handle;
		} else if (devId == SPI3_DEVICE_2) {
			selectedHandle = spi3Device2Handle;
		} else {
			selectedHandle = spi3Device3Handle;
		}
		esp_err_t err = spi_device_transmit(selectedHandle, &spi3DeviceTransactionHandle);
		xSemaphoreGive(SPI3Mutex);
		return err == ESP_OK;
	} else {
		return 0;
	}
}

