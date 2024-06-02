#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include <stdint.h>
#include <math.h>
#include "leakyIntegrationFlilter.h"
#include "lowPassFilter.h"
#include "mathUtil.h"
#include "matrixUtil.h"

/**
 * Structure representing IMU data
 */
typedef struct _IMU_DATA IMU_DATA;
struct _IMU_DATA {
	//Quaternions w, x ,y ,z
	float q0, q1, q2, q3; //z
	//Rotation matrix
	float rMatrix[3][3];
	//Euler angles
	float pitch, roll, yaw;
	//Azimuth NED heading
	float heading;
	//Motion rates
	float pitchRate, rollRate, yawRate, gRate;
	//Linear accelerations
	float linAxGRaw, linAyGRaw, linAzGRaw ;
	float linAxG, linAyG, linAzG;
	float linVz, linVx, linVy;

	uint8_t cpu;
	float dt;
};

extern IMU_DATA imuData ;
#define MAG_INCLINATION -1.41f //-1.41// 1.21?https://www.magnetic-declination.com/India/Bangalore/1132482.html
#define IMU_VEL_TRIG_APPROX_ENABLED 1
#define IMU_RM_LINEAR_ACC_ENABLED 0

uint16_t getImuStabilizationCount(void);
uint8_t imuInit(float magIncl);
void imuSetMode(uint8_t stabilize);
void imuReset(uint8_t hard);
void imuUpdate(float dt);
void setImuZAccBias(float bias);
#endif
