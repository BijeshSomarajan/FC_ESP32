#include "imu.h"
#include "attitudeSensor.h"
#include "mahonyFilter_BF.h"

IMU_DATA imuData;
extern ATTITUDE_DATA attitudeData;

float imuMagInclination = 0;
float imuZAccBias = 0.0f;

LEAKYINTEGRATIONFILTER imuZLeakyIntFilter;
LOWPASSFILTER imuLinAccXLPF, imuLinAccYLPF, imuLinAccZLPF;
LOWPASSFILTER imuLinVelZRefLPF;

void setImuZAccBias(float bias) {
	imuZAccBias = bias;
}

uint16_t getImuStabilizationCount() {
	return imuFilterGetStabilizationCount();
}

void imuSetMode(uint8_t stabilize) {
	imuFilterSetMode(stabilize);
}

#if IMU_RM_LINEAR_ACC_ENABLED == 1
//Calculates linear acceleration
void calculateLinearAcc(float dt) {
	float linAzG = (imuData.q0 * imuData.q0 - imuData.q1 * imuData.q1 - imuData.q2 * imuData.q2 + imuData.q3 * imuData.q3) * 0.98f;
	float sensorAzG = attitudeData.azG;
#if IMU_LIN_ACC_LPF_ENABLED == 1
	sensorAzG = lowPassFilterUpdate(&imuLinAccZLPF, sensorAzG, dt);

#endif
	imuData.linAzGRaw = sensorAzG - linAzG;
	imuData.linAzG = applyDeadBandFloat(0.0f,imuData.linAzGRaw - imuZAccBias, IMU_Z_ACC_DB);
	float pitchAbs = fabsf(imuData.pitch);
	float rollAbs = fabsf(imuData.roll);
	if (pitchAbs > IMU_LIN_ACC_VALID_ANGLE || rollAbs > IMU_LIN_ACC_VALID_ANGLE) {
		imuData.linAxG = 0;
		imuData.linAyG = 0;
		imuData.linAzG = 0;
		imuData.linVx = 0;
		imuData.linVy = 0;
		imuData.linVz = 0;
		leakyIntegrationFilterReset(&imuZLeakyIntFilter, 0);
	} else {
		imuData.linVz = leakyIntegrationFilterUpdate(&imuZLeakyIntFilter, imuData.linAzG, dt) * IMU_ACC_VEL_GAIN;
	}
}
#else
//Calculates linear acceleration
void calculateLinearAcc(float dt) {
	float sensorAxG = attitudeData.axG;
	float sensorAyG = attitudeData.ayG;
	float sensorAzG = attitudeData.azG;
#if IMU_LIN_ACC_LPF_ENABLED == 1
	sensorAxG = lowPassFilterUpdate(&imuLinAccXLPF, sensorAxG, dt);
	sensorAyG = lowPassFilterUpdate(&imuLinAccYLPF, sensorAyG, dt);
	sensorAzG = lowPassFilterUpdate(&imuLinAccZLPF, sensorAzG, dt);
#endif
	float axSquare = sensorAxG * sensorAxG;
	float aySquare = sensorAyG * sensorAyG;
	float azSquare = sensorAzG * sensorAzG;
#if IMU_VEL_TRIG_APPROX_ENABLED == 1
	float accRoll = atanApprox(sensorAxG / sqrtf(aySquare + azSquare));
	float accPitch = atanApprox(sensorAyG / sqrtf(axSquare + azSquare));
	float sinAccRoll = sinApprox(accRoll);
	float sinAccPitch = sinApprox(accPitch);
	float cosAccRoll = cosApprox(accRoll);
	float cosAccPitch = cosApprox(accPitch);
#else
	float accRoll = atanf(sensorAxG / sqrtf(aySquare + azSquare));
	float accPitch = atanf(sensorAyG / sqrtf(axSquare + azSquare));
	float sinAccRoll = sinf(accRoll);
	float sinAccPitch = sinf(accPitch);
	float cosAccRoll = cosf(accRoll);
	float cosAccPitch = cosf(accPitch);
#endif
	float sensorAzGFinal = sensorAxG * sinAccRoll + sensorAyG * sinAccPitch + sensorAzG * cosAccRoll * cosAccPitch;
	imuData.linAzGRaw = sensorAzGFinal - 1.0f;
	imuData.linAzG = applyDeadBandFloat(0.0f, imuData.linAzGRaw - imuZAccBias, IMU_Z_ACC_DB);
	float pitchAbs = fabsf(imuData.pitch);
	float rollAbs = fabsf(imuData.roll);
	if (pitchAbs > IMU_LIN_ACC_VALID_ANGLE || rollAbs > IMU_LIN_ACC_VALID_ANGLE) {
		imuData.linAzG = 0;
		imuData.linVz = 0;
		leakyIntegrationFilterReset(&imuZLeakyIntFilter, 0);
		//lowPassFilterResetToValue(&imuLinVelZRefLPF, 0);
	} else {
		float linActual = leakyIntegrationFilterUpdate(&imuZLeakyIntFilter, imuData.linAzG, dt) * IMU_ACC_VEL_GAIN;
		imuData.linVz = linActual;

		//float linRef = lowPassFilterUpdate(&imuLinVelZRefLPF, linActual, dt);
		//imuData.linVz = linActual - linRef;
		//imuData.linVzRef = linRef;
		//imuData.linVzAct = linActual;
	}
}
#endif

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
	leakyIntegrationFilterInit(&imuZLeakyIntFilter, IMU_ALT_VEL_BIAS_LEAK_FACTOR);
#if IMU_LIN_ACC_LPF_ENABLED == 1
	lowPassFilterInit(&imuLinAccXLPF, IMU_LIN_ACC_LPF_CUTOFF);
	lowPassFilterInit(&imuLinAccYLPF, IMU_LIN_ACC_LPF_CUTOFF);
	lowPassFilterInit(&imuLinAccZLPF, IMU_LIN_ACC_LPF_CUTOFF);
#endif
	lowPassFilterInit(&imuLinVelZRefLPF, IMU_LIN_VEL_REF_LPF_CUTOFF);
	return 1;
}

/****************************************************************************************************************/
// Resets Madgwick filter.
/****************************************************************************************************************/
void imuReset(uint8_t hard) {
	leakyIntegrationFilterReset(&imuZLeakyIntFilter, 0);
#if IMU_LIN_ACC_LPF_ENABLED == 1
	lowPassFilterResetToValue(&imuLinAccXLPF, attitudeData.axG);
	lowPassFilterResetToValue(&imuLinAccYLPF, attitudeData.ayG);
	lowPassFilterResetToValue(&imuLinAccZLPF, attitudeData.azG);
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

