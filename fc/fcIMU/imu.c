#include "include/imu.h"
#include "imuConfig.h"
#include "attitudeSensor.h"
#include "mahonyFilter_BF.h"

IMU_DATA imuData ;
extern ATTITUDE_DATA attitudeData;

float imuMagInclination = 0;
float imuXAccBias = 0.0f;
float imuYAccBias = 0.0f;
float imuZAccBias = 0.0f;

LEAKYINTEGRATIONFILTER imuXLeakyIntFilter;
LEAKYINTEGRATIONFILTER imuYLeakyIntFilter;
LEAKYINTEGRATIONFILTER imuZLeakyIntFilter;

LOWPASSFILTER imuLinAccXLPF;
LOWPASSFILTER imuLinAccYLPF;
LOWPASSFILTER imuLinAccZLPF;

float calculateLinearAccDt = 0;

void setImuXAccBias(float bias) {
	imuXAccBias = bias;
}

void setImuYAccBias(float bias) {
	imuYAccBias = bias;
}

void setImuZAccBias(float bias) {
	imuZAccBias = bias;
}

uint16_t getImuStabilizationCount() {
	return imuFilterGetStabilizationCount();
}

void imuSetMode(uint8_t stabilize) {
	imuFilterSetMode(stabilize);
}

//Calculates linear acceleration
void calculateLinearAcc(float dt) {
	float linAxG = (2 * (imuData.q1 * imuData.q3 - imuData.q0 * imuData.q2)) * 0.98f;
	float linAyG = (2 * (imuData.q0 * imuData.q1 + imuData.q2 * imuData.q3)) * 0.98f;
	float linAzG = (imuData.q0 * imuData.q0 - imuData.q1 * imuData.q1 - imuData.q2 * imuData.q2 + imuData.q3 * imuData.q3) * 0.98f;

	float sensorAxG = attitudeData.axG;
	float sensorAyG = attitudeData.ayG;
	float sensorAzG = attitudeData.azG;

#if IMU_LIN_ACC_LPF_ENABLED == 1
	sensorAxG = lowPassFilterUpdate(&imuLinAccXLPF, sensorAxG, dt);
	sensorAyG = lowPassFilterUpdate(&imuLinAccYLPF, sensorAyG, dt);
	sensorAzG = lowPassFilterUpdate(&imuLinAccZLPF, sensorAzG, dt);
#endif

	imuData.linAxGRaw = sensorAxG - linAxG;
	imuData.linAyGRaw = sensorAyG - linAyG;
	imuData.linAzGRaw = sensorAzG - linAzG;

	imuData.linAxG = applyDeadBandFloat(imuData.linAxGRaw - imuXAccBias, IMU_X_ACC_DB);
	imuData.linAyG = applyDeadBandFloat(imuData.linAyGRaw - imuYAccBias, IMU_Y_ACC_DB);
	imuData.linAzG = applyDeadBandFloat(imuData.linAzGRaw - imuZAccBias, IMU_Z_ACC_DB);

	float pitchAbs = fabsf(imuData.pitch);
	float rollAbs = fabsf(imuData.roll);

	if (pitchAbs > IMU_LIN_ACC_VALID_ANGLE || rollAbs > IMU_LIN_ACC_VALID_ANGLE) {
		imuData.linAxG = 0;
		imuData.linAyG = 0;
		imuData.linAzG = 0;

		imuData.linVx = 0;
		imuData.linVy = 0;
		imuData.linVz = 0;

		leakyIntegrationFilterReset(&imuXLeakyIntFilter, 0);
		leakyIntegrationFilterReset(&imuYLeakyIntFilter, 0);
		leakyIntegrationFilterReset(&imuZLeakyIntFilter, 0);
	} else {
		imuData.linVx = leakyIntegrationFilterUpdate(&imuXLeakyIntFilter, imuData.linAxG, dt) * IMU_ACC_VEL_GAIN;
		imuData.linVy = leakyIntegrationFilterUpdate(&imuYLeakyIntFilter, imuData.linAyG, dt) * IMU_ACC_VEL_GAIN;
		imuData.linVz = leakyIntegrationFilterUpdate(&imuZLeakyIntFilter, imuData.linAzG, dt) * IMU_ACC_VEL_GAIN;
	}
}

/*************************************************************************/
//Does imuData fusion , returns 1 if done
/*************************************************************************/
void imuUpdate(float dt) {
	imuFilterUpdate(dt);
	imuFilterUpdateAngles();
	imuFilterUpdateHeading(imuMagInclination);
	//Flush the rates, Note: X and Y gyros are interchanged
	imuData.pitchRate = attitudeData.gxDS;
	imuData.rollRate = attitudeData.gyDS;
	imuData.yawRate = attitudeData.gzDS;
	imuData.gRate = attitudeData.azG;
	calculateLinearAcc(dt);

}

/*****************************************************************************************************************/
// Initializes imuData.
/*****************************************************************************************************************/
uint8_t imuInit(float pMagInclination) {
	imuReset(1);
	imuMagInclination = pMagInclination;
	imuFilterInit(1);
	leakyIntegrationFilterInit(&imuXLeakyIntFilter, IMU_LIN_VEL_BIAS_LEAK_FACTOR);
	leakyIntegrationFilterInit(&imuYLeakyIntFilter, IMU_LIN_VEL_BIAS_LEAK_FACTOR);
	leakyIntegrationFilterInit(&imuZLeakyIntFilter, IMU_ALT_VEL_BIAS_LEAK_FACTOR);

#if IMU_LIN_ACC_LPF_ENABLED == 1
	lowPassFilterInit(&imuLinAccXLPF, IMU_LIN_ACC_LPF_CUTOFF);
	lowPassFilterInit(&imuLinAccYLPF, IMU_LIN_ACC_LPF_CUTOFF);
	lowPassFilterInit(&imuLinAccZLPF, IMU_LIN_ACC_LPF_CUTOFF);
#endif

	return 1;
}

/****************************************************************************************************************/
// Resets Madgwick filter.
/****************************************************************************************************************/
void imuReset(uint8_t hard) {
	leakyIntegrationFilterReset(&imuXLeakyIntFilter, 0);
	leakyIntegrationFilterReset(&imuYLeakyIntFilter, 0);
	leakyIntegrationFilterReset(&imuZLeakyIntFilter, 0);

#if IMU_X_LIN_ACC_LPF_ENABLED == 1
	lowPassFilterResetToValue(&imuLinAccXLPF, sensorData.axG);
	lowPassFilterResetToValue(&imuLinAccYLPF, sensorData.ayG);
	lowPassFilterResetToValue(&imuLinAccZLPF, sensorData.azG);
#endif

	if (hard) {
		//Reset the quaternion
		imuData.q0 = 1.0f;
		imuData.q1 = 0.0f;
		imuData.q2 = 0.0f;
		imuData.q3 = 0.0f;

		//Reset the rotation matrix
		imuData.rMatrix[0][0] = 0;
		imuData.rMatrix[0][1] = 0;
		imuData.rMatrix[0][2] = 0;

		imuData.rMatrix[1][0] = 0;
		imuData.rMatrix[1][1] = 0;
		imuData.rMatrix[1][2] = 0;

		imuData.rMatrix[2][0] = 0;
		imuData.rMatrix[2][1] = 0;
		imuData.rMatrix[2][2] = 0;

		//Reset the euler angles
		imuData.pitch = 0;
		imuData.roll = 0;
		imuData.yaw = 0;
		imuData.heading = 0;

		//Reset the rates
		imuData.pitchRate = 0;
		imuData.rollRate = 0;
		imuData.yawRate = 0;
		imuData.gRate = 0;

		imuData.linAxG = 0;
		imuData.linAyG = 0;
		imuData.linAzG = 0;

		imuData.linVz = 0;
		imuData.linVx = 0;
		imuData.linVy = 0;

		//Reset the specific filter
		imuFilterReset();
	} else {
		imuData.linVz = 0;
		imuData.linVx = 0;
		imuData.linVy = 0;
	}
}

