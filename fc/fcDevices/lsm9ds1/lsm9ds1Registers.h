#ifndef DEVICES_LSM9DS1REGISTERS_H_
#define DEVICES_LSM9DS1REGISTERS_H_

/////////////////////////////////////////
// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define  LSM9DS1_ACT_THS			0x04
#define  LSM9DS1_ACT_DUR			0x05
#define  LSM9DS1_INT_GEN_CFG_XL		0x06
#define  LSM9DS1_INT_GEN_THS_X_XL	0x07
#define  LSM9DS1_INT_GEN_THS_Y_XL	0x08
#define  LSM9DS1_INT_GEN_THS_Z_XL	0x09
#define  LSM9DS1_INT_GEN_DUR_XL		0x0A
#define  LSM9DS1_REFERENCE_G		0x0B
#define  LSM9DS1_INT1_CTRL			0x0C
#define  LSM9DS1_INT2_CTRL			0x0D
#define  LSM9DS1_WHO_AM_I_XG		0x0F
#define  LSM9DS1_CTRL_REG1_G		0x10
#define  LSM9DS1_CTRL_REG2_G		0x11
#define  LSM9DS1_CTRL_REG3_G		0x12
#define  LSM9DS1_ORIENT_CFG_G		0x13
#define  LSM9DS1_INT_GEN_SRC_G		0x14
#define  LSM9DS1_OUT_TEMP_L			0x15
#define  LSM9DS1_OUT_TEMP_H			0x16
#define  LSM9DS1_STATUS_REG_0		0x17
#define  LSM9DS1_OUT_X_L_G			0x18
#define  LSM9DS1_OUT_X_H_G			0x19
#define  LSM9DS1_OUT_Y_L_G			0x1A
#define  LSM9DS1_OUT_Y_H_G			0x1B
#define  LSM9DS1_OUT_Z_L_G			0x1C
#define  LSM9DS1_OUT_Z_H_G			0x1D
#define  LSM9DS1_CTRL_REG4			0x1E
#define  LSM9DS1_CTRL_REG5_XL		0x1F
#define  LSM9DS1_CTRL_REG6_XL		0x20
#define  LSM9DS1_CTRL_REG7_XL		0x21
#define  LSM9DS1_CTRL_REG8			0x22
#define  LSM9DS1_CTRL_REG9			0x23
#define  LSM9DS1_CTRL_REG10			0x24
#define  LSM9DS1_INT_GEN_SRC_XL		0x26
#define  LSM9DS1_STATUS_REG_1		0x27
#define  LSM9DS1_OUT_X_L_XL			0x28
#define  LSM9DS1_OUT_X_H_XL			0x29
#define  LSM9DS1_OUT_Y_L_XL			0x2A
#define  LSM9DS1_OUT_Y_H_XL			0x2B
#define  LSM9DS1_OUT_Z_L_XL			0x2C
#define  LSM9DS1_OUT_Z_H_XL			0x2D
#define  LSM9DS1_FIFO_CTRL			0x2E
#define  LSM9DS1_FIFO_SRC			0x2F
#define  LSM9DS1_INT_GEN_CFG_G		0x30
#define  LSM9DS1_INT_GEN_THS_XH_G	0x31
#define  LSM9DS1_INT_GEN_THS_XL_G	0x32
#define  LSM9DS1_INT_GEN_THS_YH_G	0x33
#define  LSM9DS1_INT_GEN_THS_YL_G	0x34
#define  LSM9DS1_INT_GEN_THS_ZH_G	0x35
#define  LSM9DS1_INT_GEN_THS_ZL_G	0x36
#define  LSM9DS1_INT_GEN_DUR_G		0x37

///////////////////////////////
// LSM9DS1 Magneto Registers //
///////////////////////////////
#define  LSM9DS1_OFFSET_X_REG_L_M	0x05
#define  LSM9DS1_OFFSET_X_REG_H_M	0x06
#define  LSM9DS1_OFFSET_Y_REG_L_M	0x07
#define  LSM9DS1_OFFSET_Y_REG_H_M	0x08
#define  LSM9DS1_OFFSET_Z_REG_L_M	0x09
#define  LSM9DS1_OFFSET_Z_REG_H_M	0x0A
#define  LSM9DS1_WHO_AM_I_M			0x0F
#define  LSM9DS1_CTRL_REG1_M		0x20
#define  LSM9DS1_CTRL_REG2_M		0x21
#define  LSM9DS1_CTRL_REG3_M		0x22
#define  LSM9DS1_CTRL_REG4_M		0x23
#define  LSM9DS1_CTRL_REG5_M		0x24
#define  LSM9DS1_STATUS_REG_M		0x27
#define  LSM9DS1_OUT_READ_MULTIPLE	0xC0
#define  LSM9DS1_OUT_X_L_M			0x28
#define  LSM9DS1_OUT_X_H_M			0x29
#define  LSM9DS1_OUT_Y_L_M			0x2A
#define  LSM9DS1_OUT_Y_H_M			0x2B
#define  LSM9DS1_OUT_Z_L_M			0x2C
#define  LSM9DS1_OUT_Z_H_M			0x2D
#define  LSM9DS1_INT_CFG_M			0x30
#define  LSM9DS1_INT_SRC_M			0x31
#define  LSM9DS1_INT_THS_L_M		0x32
#define  LSM9DS1_INT_THS_H_M		0x33

////////////////////////////////
// LSM9DS1 WHO_AM_I Responses //
////////////////////////////////
#define  LSM9DS1_WHO_AM_I_AG_RSP		0x68
#define  LSM9DS1_WHO_AM_I_M_RSP		0x3D

//ODR_XL2 ODR_XL1 ODR_XL0 FS1_XL FS0_XL BW_SCAL _ODR BW_XL1 BW_XL0
#define  LSM9DS1_ACCELRANGE_2G  0xC4//Acc ODR:952 , FS:2G , Filter : 408 Hz;
#define LSM9DS1_ACCELRANGE_4G  0xD4//Acc ODR:952 , FS:4G , Filter : 408 Hz;
#define LSM9DS1_ACCELRANGE_8G  0xDC//Acc ODR:952 , FS:8G , Filter : 408 Hz;
#define LSM9DS1_ACCELRANGE_16G  0xCC//Acc ODR:952 , FS:16G , Filter : 408 Hz;

#define LSM9DS1_GYRORANGE_245DPS 0xC3//Gyro ODR : 952 , FS:245 , Filter : 100 Hz
#define LSM9DS1_GYRORANGE_500DPS 0xCB//Gyro ODR : 952 , FS:500 , Filter : 100 Hz
#define LSM9DS1_GYRORANGE_2000DPS 0xDB//Gyro ODR : 952 , FS:2000 , Filter : 100 Hz

#define LSM9DS1_MAGRANGE_4GAUSS 0x0//4 Gauss
#define LSM9DS1_MAGRANGE_8GAUSS 0x20//8 Gauss
#define LSM9DS1_MAGRANGE_16GAUSS 0x60//16 Gauss

#endif
