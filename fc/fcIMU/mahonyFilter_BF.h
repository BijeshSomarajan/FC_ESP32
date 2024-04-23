#include "mathUtil.h"
#include <math.h>
#include "attitudeSensor.h"

#define MAHONY_BF_FILTER_BF_SPIN_RATE_LIMIT 10
#define MAHONY_BF_FILTER_BF_USE_TRIG_APPROX 1
#define MAHONY_BF_FILTER_BF_USE_SQRT_APPROX 1

#define MAHONY_BF_FILTER_BF_FLOOR_ANGLES 0
//Higher value means less reliability on gyro
#define MAHONY_BF_FILTER_KP  0.275f//0.25f  //100.7f; //0.155f; //0.175f; //0.225 //0.125f; //0.235
#define MAHONY_BF_FILTER_KI  0.1f
#define MAHONY_BF_FILTER_STABILIZE_KP  MAHONY_BF_FILTER_KP * 10.0f //1.75
#define MAHONY_BF_FILTER_STABILIZE_KI  MAHONY_BF_FILTER_KI * 10.0f
#define MAHONY_BF_FILTER_STAB_COUNT 5000

typedef struct {
	float ww, wx, wy, wz, xx, xy, xz, yy, yz, zz;
} MahonyFilter_BF_QuaternionProducts;

void imuFilterUpdate(float dt);
void imuFilterSetMode(uint8_t stablize);
uint8_t imuFilterInit(uint8_t stabilize);
void imuFilterReset(void);
uint16_t imuFilterGetStabilizationCount(void);
void imuFilterSetMode(uint8_t stablize);
void imuFilterUpdateAngles(void);
void imuFilterUpdateHeading(float magIncl);
