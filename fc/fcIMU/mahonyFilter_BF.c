#include "mahonyFilter_BF.h"
#include "include/imu.h"

float mahonyFilter_BF_KP = MAHONY_BF_FILTER_KP;
float mahonyFilter_BF_KI = 0;

MahonyFilter_BF_QuaternionProducts mahonyFilter_BF_Qp = { .ww = 1, .wx = 0, .wy = 0, .wz = 0, .xx = 0, .xy = 0, .xz = 0, .yy = 0, .yz = 0, .zz = 0 };

// Integral error terms
float mahonyFilter_BF_IBx = 0.0f;
float mahonyFilter_BF_IBy = 0.0f;
float mahonyFilter_BF_IBz = 0.0f;

uint16_t imuFilterGetStabilizationCount() {
	return MAHONY_BF_FILTER_STAB_COUNT;
}

void imuFilterUpdateAngles(void) {
#if MAHONY_BF_FILTER_BF_USE_TRIG_APPROX == 1
	imuData.roll = convertRadToDeg(atan2Approx(imuData.rMatrix[2][1], imuData.rMatrix[2][2]));
	imuData.pitch = convertRadToDeg(HalfPI - acosApprox(-imuData.rMatrix[2][0]));
	imuData.yaw = convertRadToDeg(-atan2Approx(imuData.rMatrix[1][0], imuData.rMatrix[0][0]));
# else
	imuData.roll = convertRadToDeg(atan2f(imuData.rMatrix[2][1], imuData.rMatrix[2][2]));
	imuData.pitch = convertRadToDeg(HalfPI - acosf(-imuData.rMatrix[2][0]));
	imuData.yaw = convertRadToDeg(-atan2f(imuData.rMatrix[1][0], imuData.rMatrix[0][0]));
#endif

#if MAHONY_BF_FILTER_BF_FLOOR_ANGLES == 1
	imuData.roll = lrintf(imuData.roll);
	imuData.pitch = lrintf(imuData.pitch);
	imuData.yaw = lrintf(imuData.yaw);
#endif

}

/**
 * Converts yaw to heading
 */
void imuFilterUpdateHeading(float magIncl) {
	imuData.heading = -imuData.yaw - 90.0f;
	if (imuData.heading < 0.0f) {
		imuData.heading += 360.0f;
	} else if (imuData.heading > 360.0f) {
		imuData.heading -= 360.0f;
	}
	imuData.heading += magIncl;
	if (imuData.heading > 360) {
		imuData.heading -= 360;
	} else if (imuData.heading < 0) {
		imuData.heading += 360;
	}
}

void mahonyFilter_BF_QuaternionComputeProducts() {
	mahonyFilter_BF_Qp.ww = power2f(imuData.q0);
	mahonyFilter_BF_Qp.wx = imuData.q0 * imuData.q1;
	mahonyFilter_BF_Qp.wy = imuData.q0 * imuData.q2;
	mahonyFilter_BF_Qp.wz = imuData.q0 * imuData.q3;
	mahonyFilter_BF_Qp.xx = power2f(imuData.q1);
	mahonyFilter_BF_Qp.xy = imuData.q1 * imuData.q2;
	mahonyFilter_BF_Qp.xz = imuData.q1 * imuData.q3;
	mahonyFilter_BF_Qp.yy = power2f(imuData.q2);
	mahonyFilter_BF_Qp.yz = imuData.q2 * imuData.q3;
	mahonyFilter_BF_Qp.zz = power2f(imuData.q3);
}

void mahonyFilter_BF_ComputeRotationMatrix(void) {
	mahonyFilter_BF_QuaternionComputeProducts();

	imuData.rMatrix[0][0] = 1.0f - (2.0f * mahonyFilter_BF_Qp.yy) - (2.0f * mahonyFilter_BF_Qp.zz);
	imuData.rMatrix[0][1] = 2.0f * (mahonyFilter_BF_Qp.xy - mahonyFilter_BF_Qp.wz);
	imuData.rMatrix[0][2] = 2.0f * (mahonyFilter_BF_Qp.xz + mahonyFilter_BF_Qp.wy);

	imuData.rMatrix[1][0] = 2.0f * (mahonyFilter_BF_Qp.xy + mahonyFilter_BF_Qp.wz);
	imuData.rMatrix[1][1] = 1.0f - (2.0f * mahonyFilter_BF_Qp.xx) - (2.0f * mahonyFilter_BF_Qp.zz);
	imuData.rMatrix[1][2] = 2.0f * (mahonyFilter_BF_Qp.yz - mahonyFilter_BF_Qp.wx);

	imuData.rMatrix[2][0] = 2.0f * (mahonyFilter_BF_Qp.xz - mahonyFilter_BF_Qp.wy);
	imuData.rMatrix[2][1] = 2.0f * (mahonyFilter_BF_Qp.yz + mahonyFilter_BF_Qp.wx);
	imuData.rMatrix[2][2] = 1.0f - (2.0f * mahonyFilter_BF_Qp.xx) - (2.0f * mahonyFilter_BF_Qp.yy);
}

void imuFilterUpdate(float dt) {
	//Copy the variables locally
	float gx = convertDegToRad(attitudeData.gxDS);
	float gy = convertDegToRad(attitudeData.gyDS);
	float gz = convertDegToRad(attitudeData.gzDS);
	float ax = attitudeData.axG;
	float ay = attitudeData.ayG;
	float az = attitudeData.azG;
	float mx = attitudeData.mx;
	float my = attitudeData.my;
	float mz = attitudeData.mz;
	float halfDt = 0.5f * dt;

	// Use raw heading error (from GPS or whatever else)
	float ex = 0, ey = 0, ez = 0;

	// Use measured magnetic field vector
	float recipMagNorm = power2f(mx) + power2f(my) + power2f(mz);
	if (recipMagNorm > 0.001f) {
		// Normalize magnetometer measurement
#if MAHONY_BF_FILTER_BF_USE_SQRT_APPROX == 1
		recipMagNorm = fastInvSqrtf(recipMagNorm);
#else
		recipMagNorm = invSqrtf(recipMagNorm);
#endif
		mx *= recipMagNorm;
		my *= recipMagNorm;
		mz *= recipMagNorm;
		// For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
		// This way magnetic field will only affect heading and wont mess roll/pitch angles
		// (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
		// (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
		const float hx = imuData.rMatrix[0][0] * mx + imuData.rMatrix[0][1] * my + imuData.rMatrix[0][2] * mz;
		const float hy = imuData.rMatrix[1][0] * mx + imuData.rMatrix[1][1] * my + imuData.rMatrix[1][2] * mz;
		const float bx = sqrtf(hx * hx + hy * hy);
		// magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
		float ez_ef = -(hy * bx);
		// Rotate mag error vector back to BF and accumulate
		ex += (imuData.rMatrix[2][0] * ez_ef);
		ey += (imuData.rMatrix[2][1] * ez_ef);
		ez += (imuData.rMatrix[2][2] * ez_ef);
	}
	// Use measured acceleration vector
	float recipAccNorm = power2f(ax) + power2f(ay) + power2f(az);
	if (recipAccNorm > 0.001f) {
		// Normalize accelerometer measurement
#if MAHONY_BF_FILTER_BF_USE_SQRT_APPROX == 1
		recipMagNorm = fastInvSqrtf(recipMagNorm);
#else
		recipAccNorm = invSqrtf(recipAccNorm);
#endif
		ax *= recipAccNorm;
		ay *= recipAccNorm;
		az *= recipAccNorm;
		// Error is sum of cross product between estimated direction and measured direction of gravity
		ex += ((ay * imuData.rMatrix[2][2]) - (az * imuData.rMatrix[2][1]));
		ey += ((az * imuData.rMatrix[2][0]) - (ax * imuData.rMatrix[2][2]));
		ez += ((ax * imuData.rMatrix[2][1]) - (ay * imuData.rMatrix[2][0]));
	}

	// Compute and apply integral feedback if enabled
	if (mahonyFilter_BF_KI > 0.0f) {
		// Calculate general spin rate (rad/s)
		float spin_rate = sqrtf(power2f(gx) + power2f(gy) + power2f(gz));
		// Stop integrating if spinning beyond the certain limit
		if (spin_rate < convertDegToRad(MAHONY_BF_FILTER_BF_SPIN_RATE_LIMIT)) {
			mahonyFilter_BF_IBx += (mahonyFilter_BF_KI * ex * dt);    // integral error scaled by Ki
			mahonyFilter_BF_IBy += (mahonyFilter_BF_KI * ey * dt);
			mahonyFilter_BF_IBz += (mahonyFilter_BF_KI * ez * dt);
		}
	} else {
		mahonyFilter_BF_IBx = 0.0f;    // prevent integral windup
		mahonyFilter_BF_IBy = 0.0f;
		mahonyFilter_BF_IBz = 0.0f;
	}

	// Apply proportional and integral feedback
	gx += ((mahonyFilter_BF_KP * ex) + mahonyFilter_BF_IBx);
	gy += ((mahonyFilter_BF_KP * ey) + mahonyFilter_BF_IBy);
	gz += ((mahonyFilter_BF_KP * ez) + mahonyFilter_BF_IBz);

	// Integrate rate of change of quaternion
	gx *= halfDt;
	gy *= halfDt;
	gz *= halfDt;

	//Compute the new quaternion
	imuData.q0 += (-(imuData.q1 * gx) - (imuData.q2 * gy) - (imuData.q3 * gz));
	imuData.q1 += ((imuData.q0 * gx) + (imuData.q2 * gz) - (imuData.q3 * gy));
	imuData.q2 += ((imuData.q0 * gy) - (imuData.q1 * gz) + (imuData.q3 * gx));
	imuData.q3 += ((imuData.q0 * gz) + (imuData.q1 * gy) - (imuData.q2 * gx));

	// Normalize quaternion
#if MAHONY_BF_FILTER_BF_USE_SQRT_APPROX == 1
	float recipNorm = fastInvSqrtf(power2f(imuData.q0) + power2f(imuData.q1) + power2f(imuData.q2) + power2f(imuData.q3));
#else
	float recipNorm = invSqrtf(power2f(imuData.q0) + power2f(imuData.q1) + power2f(imuData.q2) + power2f(imuData.q3));
#endif

	imuData.q0 *= recipNorm;
	imuData.q1 *= recipNorm;
	imuData.q2 *= recipNorm;
	imuData.q3 *= recipNorm;

	// Pre-compute rotation matrix from quaternion
	mahonyFilter_BF_ComputeRotationMatrix();
}

void imuFilterSetMode(uint8_t stablize) {
	if (stablize) {
		mahonyFilter_BF_KP = MAHONY_BF_FILTER_STABILIZE_KP;
		mahonyFilter_BF_KI = MAHONY_BF_FILTER_STABILIZE_KI;
	} else {
		mahonyFilter_BF_KP = MAHONY_BF_FILTER_KP;
		mahonyFilter_BF_KI = MAHONY_BF_FILTER_KI;
	}
}

uint8_t imuFilterInit(uint8_t stabilize) {
	imuFilterSetMode(stabilize);
	imuFilterReset();
	mahonyFilter_BF_ComputeRotationMatrix();
	return 1;
}

void imuFilterReset() {
	mahonyFilter_BF_Qp.ww = 1;
	mahonyFilter_BF_Qp.wx = 0;
	mahonyFilter_BF_Qp.wy = 0;
	mahonyFilter_BF_Qp.wz = 0;
	mahonyFilter_BF_Qp.xx = 0;
	mahonyFilter_BF_Qp.xy = 0;
	mahonyFilter_BF_Qp.xz = 0;
	mahonyFilter_BF_Qp.yy = 0;
	mahonyFilter_BF_Qp.yz = 0;
	mahonyFilter_BF_Qp.zz = 0;

	mahonyFilter_BF_IBx = 0.0f;
	mahonyFilter_BF_IBy = 0.0f;
	mahonyFilter_BF_IBz = 0.0f;
}
