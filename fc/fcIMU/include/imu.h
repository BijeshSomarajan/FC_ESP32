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
	float linAxGRaw, linAyGRaw, linAzGRaw;
	float linAxG, linAyG, linAzG;
	float linVz, linVx, linVy;

	uint8_t cpu;
	float dt;
};

extern IMU_DATA imuData ;
uint16_t getImuStabilizationCount(void);
uint8_t imuInit(float magIncl);
void imuSetMode(uint8_t stabilize);
void imuReset(uint8_t hard);
void imuUpdate(float dt);

void setImuXAccBias(float bias);
void setImuYAccBias(float bias);
void setImuZAccBias(float bias);

#define MAG_INCLINATION -1.21f //-1.41// 1.21?https://www.magnetic-declination.com/India/Bangalore/1132482.html

#endif
