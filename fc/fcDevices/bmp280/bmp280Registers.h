// BMP280 registers
#define BMP280_TEMP_XLSB  0xFC
#define BMP280_TEMP_LSB   0xFB
#define BMP280_TEMP_MSB   0xFA
#define BMP280_PRESS_XLSB 0xF9
#define BMP280_PRESS_LSB  0xF8
#define BMP280_PRESS_MSB  0xF7
#define BMP280_CONFIG     0xF5
#define BMP280_CTRL_MEAS  0xF4
#define BMP280_STATUS     0xF3
#define BMP280_RESET      0xE0
#define BMP280_WHO_AM_I       0xD0  
#define BMP280_WHO_AM_I_RETURN   0x58  
#define BMP280_CALIB00    0x88

enum BMP280IIRFilter {
  Off   = 0,  
  FC_2  = 1,
  FC_4  = 2,
  FC_8  = 4,
  FC_16 = 5
};

enum BMP280Mode {
  Sleep    = 0,
  Forced   = 1,
  Fforced2 = 2,
  Normal   = 3
};

enum BMP280SBy {
  t_00_5ms = 0,
  t_62_5ms = 1,
  t_125ms  = 2,
  t_250ms  = 3,
  t_500ms  = 4,
  t_1000ms = 5,
  t_2000ms = 6,
  t_4000ms = 7
};

enum BMP280Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01 = 1,
  P_OSR_02 = 2,
  P_OSR_04 = 3,
  P_OSR_08 = 4,
  P_OSR_16 = 5
};

enum BMP280Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01 = 1,
  T_OSR_02 = 2,
  T_OSR_04 = 3,
  T_OSR_08 = 4,
  T_OSR_16 = 5
};
