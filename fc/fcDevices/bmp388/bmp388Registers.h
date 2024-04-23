#ifndef _BMP388_REGISTERS_H_
#define _BMP388_REGISTERS_H_

/**\name BMP3 chip identifier */
#define BMP3_CHIP_ID   (0x50)

/**\name Register Address */
#define BMP3_CHIP_ID_ADDR     (0x00)
#define BMP3_ERR_REG_ADDR     (0x02)
#define BMP3_SENS_STATUS_REG_ADDR   (0x03)
#define BMP3_DATA_ADDR     (0x04)
#define BMP3_EVENT_ADDR     (0x10)
#define BMP3_INT_STATUS_REG_ADDR   (0x11)
#define BMP3_FIFO_LENGTH_ADDR     (0x12)
#define BMP3_FIFO_DATA_ADDR     (0x14)
#define BMP3_FIFO_WM_ADDR     (0x15)
#define BMP3_CONFIG_ADDR   (0x1F)
#define BMP3_FIFO_CONFIG_1_ADDR   (0x17)
#define BMP3_FIFO_CONFIG_2_ADDR   (0x18)
#define BMP3_INT_CTRL_ADDR     (0x19)
#define BMP3_IF_CONF_ADDR     (0x1A)
#define BMP3_PWR_CTRL_ADDR     (0x1B)
#define BMP3_OSR_ADDR       (0X1C)
#define BMP3_ODR_ADDR       (0X1D)
#define BMP3_CALIB_DATA_ADDR     (0x31)
#define BMP3_CMD_ADDR       (0x7E)

/**\name Power mode macros */
#define BMP3_SLEEP_MODE      (0x00 << 4)
#define BMP3_FORCED_MODE     (0x01 << 4)
#define BMP3_NORMAL_MODE     (0x03 << 4 )
#define BMP3_PRESSURE_ENABLE     (0x01)
#define BMP3_TEMPERATURE_ENABLE     (0x02)

/**\name FIFO Sub-sampling macros */
#define BMP3_FIFO_NO_SUBSAMPLING     (0x00)
#define BMP3_FIFO_SUBSAMPLING_2X     (0x01)
#define BMP3_FIFO_SUBSAMPLING_4X     (0x02)
#define BMP3_FIFO_SUBSAMPLING_8X     (0x03)
#define BMP3_FIFO_SUBSAMPLING_16X     (0x04)
#define BMP3_FIFO_SUBSAMPLING_32X     (0x05)
#define BMP3_FIFO_SUBSAMPLING_64X     (0x06)
#define BMP3_FIFO_SUBSAMPLING_128X     (0x07)

/**\name Over sampling macros */
#define BMP3_NO_OVERSAMPLING_1X     (0x00)
#define BMP3_OVERSAMPLING_2X     (0x01)
#define BMP3_OVERSAMPLING_4X     (0x02)
#define BMP3_OVERSAMPLING_8X     (0x03)
#define BMP3_OVERSAMPLING_16X     (0x04)
#define BMP3_OVERSAMPLING_32X     (0x05)

/**\name Filter setting macros */
#define BMP3_IIR_FILTER_DISABLE     (0x00)
#define BMP3_IIR_FILTER_COEFF_1     (0x01)
#define BMP3_IIR_FILTER_COEFF_3     (0x02)
#define BMP3_IIR_FILTER_COEFF_7     (0x03)
#define BMP3_IIR_FILTER_COEFF_15     (0x04)
#define BMP3_IIR_FILTER_COEFF_31     (0x05)
#define BMP3_IIR_FILTER_COEFF_63     (0x06)
#define BMP3_IIR_FILTER_COEFF_127     (0x07)

/**\name Odr setting macros */
#define BMP3_ODR_200_HZ     (0x00)
#define BMP3_ODR_100_HZ     (0x01)
#define BMP3_ODR_50_HZ     (0x02)
#define BMP3_ODR_25_HZ     (0x03)
#define BMP3_ODR_12_5_HZ     (0x04)
#define BMP3_ODR_6_25_HZ     (0x05)
#define BMP3_ODR_3_1_HZ     (0x06)
#define BMP3_ODR_1_5_HZ     (0x07)
#define BMP3_ODR_0_78_HZ     (0x08)
#define BMP3_ODR_0_39_HZ     (0x09)
#define BMP3_ODR_0_2_HZ     (0x0A)
#define BMP3_ODR_0_1_HZ     (0x0B)
#define BMP3_ODR_0_05_HZ     (0x0C)
#define BMP3_ODR_0_02_HZ     (0x0D)
#define BMP3_ODR_0_01_HZ     (0x0E)
#define BMP3_ODR_0_006_HZ     (0x0F)
#define BMP3_ODR_0_003_HZ     (0x10)
#define BMP3_ODR_0_001_HZ     (0x11)

#define BMP3_PWR_SOFT_RESET     (0xB6)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BMP3_CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BMP3_CALIB_DATA_LEN 21

#endif
