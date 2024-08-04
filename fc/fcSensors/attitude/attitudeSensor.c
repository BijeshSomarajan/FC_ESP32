#include "attitudeSensor.h"
#include "deltaTimer.h"
#include "calibration.h"
#include "biQuadFilter.h"
#include "lowPassFilter.h"
#include "indicator.h"
#include "configSensor.h"
#include "delayTimer.h"

MEMSDATA memsData;
ATTITUDE_DATA attitudeData;

//Calibration related
LOWPASSFILTER sensorAccXCalibLPF, sensorAccYCalibLPF, sensorAccZCalibLPF;
LOWPASSFILTER sensorGyroXCalibLPF, sensorGyroYCalibLPF, sensorGyroZCalibLPF;
LOWPASSFILTER sensorTempCalibLPF;
int32_t sensorAttitudeTempCalibData[9];

void resetAttitudeSensors() {
	attitudeData.axG = 0;
	attitudeData.ayG = 0;
	attitudeData.azG = 0;
	attitudeData.gxDS = 0;
	attitudeData.gyDS = 0;
	attitudeData.gzDS = 0;
	attitudeData.axGRaw = 0;
	attitudeData.ayGRaw = 0;
	attitudeData.azGRaw = 0;
	attitudeData.gxDSRaw = 0;
	attitudeData.gyDSRaw = 0;
	attitudeData.gzDSRaw = 0;
	attitudeData.mx = 0;
	attitudeData.my = 0;
	attitudeData.mz = 0;
	attitudeData.temp = 0;
	attitudeData.tempRaw = 0;
}

void loadAttitudeSensorConfig() {
	memsData.offsetAx = getCalibrationValue(CALIB_PROP_AX_BIAS_ADDR);
	memsData.offsetAy = getCalibrationValue(CALIB_PROP_AY_BIAS_ADDR);
	memsData.offsetAz = getCalibrationValue(CALIB_PROP_AZ_BIAS_ADDR);

	memsData.offsetGx = getCalibrationValue(CALIB_PROP_GX_BIAS_ADDR);
	memsData.offsetGy = getCalibrationValue(CALIB_PROP_GY_BIAS_ADDR);
	memsData.offsetGz = getCalibrationValue(CALIB_PROP_GZ_BIAS_ADDR);

	memsData.offsetMx = getScaledCalibrationValue(CALIB_PROP_MX_OFFSET_ADDR);
	memsData.offsetMy = getScaledCalibrationValue(CALIB_PROP_MY_OFFSET_ADDR);
	memsData.offsetMz = getScaledCalibrationValue(CALIB_PROP_MZ_OFFSET_ADDR);

	memsData.biasMx = getScaledCalibrationValue(CALIB_PROP_MX_BIAS_ADDR);
	memsData.biasMy = getScaledCalibrationValue(CALIB_PROP_MY_BIAS_ADDR);
	memsData.biasMz = getScaledCalibrationValue(CALIB_PROP_MZ_BIAS_ADDR);

	memsData.scaleMx = getScaledCalibrationValue(CALIB_PROP_MX_SCALE_ADDR);
	memsData.scaleMy = getScaledCalibrationValue(CALIB_PROP_MY_SCALE_ADDR);
	memsData.scaleMz = getScaledCalibrationValue(CALIB_PROP_MZ_SCALE_ADDR);

	memsData.offsetTemp = getScaledCalibrationValue(CALIB_PROP_IMU_TEMP_ADDR);
	memsData.tempC = memsData.offsetTemp;

	//Higher order coefficients are divided by higher powers of 10
	memsData.accXTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C_ADDR) / 10000.0f; //3136.34291;
	memsData.accXTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C1_ADDR) / 100000.0f; //-66.95038;
	memsData.accXTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C2_ADDR) / 1000000.0f; //-3.85809;
	memsData.accXTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C3_ADDR) / 10000000.0f; //0.04168;

	memsData.accYTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C_ADDR) / 10000.0f; //3099.9199;
	memsData.accYTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C1_ADDR) / 100000.0f; //-111.3936;
	memsData.accYTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C2_ADDR) / 1000000.0f; //-1.23488;
	memsData.accYTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C3_ADDR) / 10000000.0f; //0.00965;

	memsData.accZTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C_ADDR) / 10000.0f; //-14301.91832;
	memsData.accZTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C1_ADDR) / 100000.0f; //6.38;
	memsData.accZTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C2_ADDR) / 1000000.0f; //-4.84;
	memsData.accZTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C3_ADDR) / 10000000.0f; //0.03351;

	memsData.gyroXTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C_ADDR) / 10000.0f; //12.8099;
	memsData.gyroXTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C1_ADDR) / 100000.0f; //-1.85993;
	memsData.gyroXTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C2_ADDR) / 1000000.0f; //0.4426;
	memsData.gyroXTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C3_ADDR) / 10000000.0f; //-0.00890;

	memsData.gyroYTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C_ADDR) / 10000.0f; //10.94848;
	memsData.gyroYTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C1_ADDR) / 100000.0f; //-.46218;
	memsData.gyroYTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C2_ADDR) / 1000000.0f; //0.20199;
	memsData.gyroYTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C3_ADDR) / 10000000.0f; //-0.002;

	memsData.gyroZTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C_ADDR) / 10000.0f; //181.0141
	memsData.gyroZTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C1_ADDR) / 100000.0f; //-6.10440;
	memsData.gyroZTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C2_ADDR) / 1000000.0f; //-0.608;
	memsData.gyroZTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C3_ADDR) / 10000000.0f; //0.00962;
}

uint8_t initAttitudeSensors(float motorKv, float batVolt, float nMotor, float nPropPairs) {
	uint8_t status = memsInit();
	if (status) {
		loadAttitudeSensorConfig();
		logString("[attitude] Calibration > Loaded\n");

		//Low pass filters for Calibration
		lowPassFilterInit(&sensorAccXCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorAccYCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorAccZCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);

		lowPassFilterInit(&sensorGyroXCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorGyroYCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorGyroZCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);

		lowPassFilterInit(&sensorTempCalibLPF, SENSOR_AG_TEMP_CALIB_LOWPASS_FREQ);
		logString("[attitude] > Filters initialized Success\n");
		logString("[attitude] > Success\n");
	} else {
		logString("[attitude] > Failed\n");
	}
	return status;
}

void processAccSensorData(float dt) {
	memsApplyAccOffsetCorrection();
	memsApplyAccDataScaling();
	memsApplyAccTempOffsetCorrection();

	attitudeData.axGRaw = memsData.axG;
	attitudeData.ayGRaw = memsData.ayG;
	attitudeData.azGRaw = (1 + memsData.azG);

	attitudeData.axG = memsData.axG;
	attitudeData.ayG = memsData.ayG;
	attitudeData.azG = (1 + memsData.azG);

	//Limit the values
	attitudeData.axG = constrainToRangeF(attitudeData.axG, -SENSOR_ACC_FLYABLE_VALUE_XY_LIMIT, SENSOR_ACC_FLYABLE_VALUE_XY_LIMIT);
	attitudeData.ayG = constrainToRangeF(attitudeData.ayG, -SENSOR_ACC_FLYABLE_VALUE_XY_LIMIT, SENSOR_ACC_FLYABLE_VALUE_XY_LIMIT);
	attitudeData.azG = constrainToRangeF(attitudeData.azG, -SENSOR_ACC_FLYABLE_VALUE_Z_LIMIT, SENSOR_ACC_FLYABLE_VALUE_Z_LIMIT);
}

float getMaxValidG() {
	return getMemsMaxValidG();
}

void processGyroSensorData(float dt) {
	memsApplyGyroOffsetCorrection();
	memsApplyGyroDataScaling();
	memsApplyGyroTempOffsetCorrection();

	attitudeData.gxDSRaw = memsData.gxDS;
	attitudeData.gyDSRaw = memsData.gyDS;
	attitudeData.gzDSRaw = memsData.gzDS;

	attitudeData.gxDS = -memsData.gxDS;
	attitudeData.gyDS = -memsData.gyDS;
	attitudeData.gzDS = -memsData.gzDS;

	//Limit the values
	attitudeData.gxDS = constrainToRangeF(attitudeData.gxDS, -SENSOR_GYRO_FLYABLE_VALUE_LIMIT, SENSOR_GYRO_FLYABLE_VALUE_LIMIT);
	attitudeData.gyDS = constrainToRangeF(attitudeData.gyDS, -SENSOR_GYRO_FLYABLE_VALUE_LIMIT, SENSOR_GYRO_FLYABLE_VALUE_LIMIT);
	attitudeData.gzDS = constrainToRangeF(attitudeData.gzDS, -SENSOR_GYRO_FLYABLE_VALUE_LIMIT, SENSOR_GYRO_FLYABLE_VALUE_LIMIT);
}

void readAccSensor(float dt) {
	memsReadAcc();
	processAccSensorData(dt);
}

void readGyroSensor(float dt) {
	memsReadGyro();
	processGyroSensorData(dt);
}

void readAccAndGyroSensor(float dt) {
	memsReadAccAndGyro();
	processAccSensorData(dt);
	processGyroSensorData(dt);
}

/**
 Calculates the temperature offsets by 3rd polynomial
 **/

void readTempSensor(float dt) {
	memsReadTemp();
	memsApplyTempDataScaling();
}

void readMagSensor(float dt) {
	memsReadMag();
	memsApplyMagDataScaling();
	memsApplyMagOffsetCorrection();
}

void calculateAccAndGyroBias() {
	//Calibrate mems Acc and Gyro
	memsData.offsetAx = 0;
	memsData.offsetAy = 0;
	memsData.offsetAz = 0;
	memsData.offsetGx = 0;
	memsData.offsetGy = 0;
	memsData.offsetGz = 0;
	uint8_t lpfInit = 0;
	//Take average of Acc and Gyro readings
	for (int16_t sampleCount = 0; sampleCount < SENSOR_AG_OFFSET_CALIB_SAMPLE_COUNT; sampleCount++) {
		memsReadAccAndGyro();
		memsReadTemp();
		memsApplyTempDataScaling();
		if (!lpfInit) {
			lowPassFilterResetToValue(&sensorAccXCalibLPF, memsData.rawAx);
			lowPassFilterResetToValue(&sensorAccYCalibLPF, memsData.rawAy);
			lowPassFilterResetToValue(&sensorAccZCalibLPF, memsData.rawAz);
			lowPassFilterResetToValue(&sensorGyroXCalibLPF, memsData.rawGx);
			lowPassFilterResetToValue(&sensorGyroYCalibLPF, memsData.rawGy);
			lowPassFilterResetToValue(&sensorGyroZCalibLPF, memsData.rawGz);
			lowPassFilterResetToValue(&sensorTempCalibLPF, memsData.tempC);
			lpfInit = 1;
		}
		float dt = SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY * 0.001;
		memsData.offsetAx = lowPassFilterUpdate(&sensorAccXCalibLPF, memsData.rawAx, dt);
		memsData.offsetAy = lowPassFilterUpdate(&sensorAccYCalibLPF, memsData.rawAy, dt);
		memsData.offsetAz = lowPassFilterUpdate(&sensorAccZCalibLPF, memsData.rawAz, dt);
		memsData.offsetGx = lowPassFilterUpdate(&sensorGyroXCalibLPF, memsData.rawGx, dt);
		memsData.offsetGy = lowPassFilterUpdate(&sensorGyroYCalibLPF, memsData.rawGy, dt);
		memsData.offsetGz = lowPassFilterUpdate(&sensorGyroZCalibLPF, memsData.rawGz, dt);
		memsData.offsetTemp = lowPassFilterUpdate(&sensorTempCalibLPF, memsData.tempC, dt);
		delayMs(SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY);
		if (sampleCount % 10 == 0) {
			statusIndicatorToggle();
		}
	}
	//Back fill data for persistence
	setCalibrationValue(CALIB_PROP_AX_BIAS_ADDR, memsData.offsetAx);
	setCalibrationValue(CALIB_PROP_AY_BIAS_ADDR, memsData.offsetAy);
	setCalibrationValue(CALIB_PROP_AZ_BIAS_ADDR, memsData.offsetAz);
	setCalibrationValue(CALIB_PROP_GX_BIAS_ADDR, memsData.offsetGx);
	setCalibrationValue(CALIB_PROP_GY_BIAS_ADDR, memsData.offsetGy);
	setCalibrationValue(CALIB_PROP_GZ_BIAS_ADDR, memsData.offsetGz);
	setCalibrationValue(CALIB_PROP_IMU_TEMP_ADDR, getCalibrationScalableValue(memsData.offsetTemp));
	//Persist the calibration
	saveCalibration();
}

void calculateMagBias() {
	memsData.offsetMx = 0;
	memsData.offsetMy = 0;
	memsData.offsetMz = 0;
	//Determining magnetometer bias , Move the device in 8 pattern
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 };
	for (int indx = 0; indx < SENSOR_MAG_CALIB_SAMPLE_COUNT; indx++) {
		// Read the mag data
		memsReadMag();
		if (memsData.rawMx > mag_max[0]) {
			mag_max[0] = memsData.rawMx;
		} else if (memsData.rawMx < mag_min[0]) {
			mag_min[0] = memsData.rawMx;
		}
		if (memsData.rawMy > mag_max[1]) {
			mag_max[1] = memsData.rawMy;
		} else if (memsData.rawMy < mag_min[1]) {
			mag_min[1] = memsData.rawMy;
		}
		if (memsData.rawMz > mag_max[2]) {
			mag_max[2] = memsData.rawMz;
		} else if (memsData.rawMz < mag_min[2]) {
			mag_min[2] = memsData.rawMz;
		}
		delayMs(SENSOR_MAG_CALIB_SAMPLE_DELAY);
		statusIndicatorToggle();
	}
	// Get hard iron correction , Bias
	memsData.biasMx = ((float) (mag_max[0] + mag_min[0]) / 2.0f) * memsData.magSensitivity;
	memsData.biasMy = ((float) (mag_max[1] + mag_min[1]) / 2.0f) * memsData.magSensitivity;
	memsData.biasMz = ((float) (mag_max[2] + mag_min[2]) / 2.0f) * memsData.magSensitivity;
	// Get soft iron correction estimate
	memsData.scaleMx = ((float) (mag_max[0] - mag_min[0])) / 2.0f; // get average x axis max chord length in counts
	memsData.scaleMy = ((float) (mag_max[1] - mag_min[1])) / 2.0f; // get average y axis max chord length in counts
	memsData.scaleMz = ((float) (mag_max[2] - mag_min[2])) / 2.0f; // get average z axis max chord length in counts
	float avg_rad = (memsData.scaleMx + memsData.scaleMy + memsData.scaleMz) / 3.0f;
	memsData.scaleMx = avg_rad / memsData.scaleMx;
	memsData.scaleMy = avg_rad / memsData.scaleMy;
	memsData.scaleMz = avg_rad / memsData.scaleMz;
	//Back fill data for persistence
	setCalibrationValue(CALIB_PROP_MX_OFFSET_ADDR, getCalibrationScalableValue(memsData.offsetMx));
	setCalibrationValue(CALIB_PROP_MY_OFFSET_ADDR, getCalibrationScalableValue(memsData.offsetMy));
	setCalibrationValue(CALIB_PROP_MZ_OFFSET_ADDR, getCalibrationScalableValue(memsData.offsetMz));

	setCalibrationValue(CALIB_PROP_MX_BIAS_ADDR, getCalibrationScalableValue(memsData.biasMx));
	setCalibrationValue(CALIB_PROP_MY_BIAS_ADDR, getCalibrationScalableValue(memsData.biasMy));
	setCalibrationValue(CALIB_PROP_MZ_BIAS_ADDR, getCalibrationScalableValue(memsData.biasMz));

	setCalibrationValue(CALIB_PROP_MX_SCALE_ADDR, getCalibrationScalableValue(memsData.scaleMx));
	setCalibrationValue(CALIB_PROP_MY_SCALE_ADDR, getCalibrationScalableValue(memsData.scaleMy));
	setCalibrationValue(CALIB_PROP_MZ_SCALE_ADDR, getCalibrationScalableValue(memsData.scaleMz));

	//Persist the calibration
	saveCalibration();
}

/***********************************************************************/
/* Send the calibration data                                           */
/***********************************************************************/
void sendAttitudeTempCalibData() {
	sensorAttitudeTempCalibData[0] = memsData.tempC * 100000;
	sensorAttitudeTempCalibData[1] = memsData.axG * 100000;
	sensorAttitudeTempCalibData[2] = memsData.ayG * 100000;
	sensorAttitudeTempCalibData[3] = memsData.azG * 100000;
	sensorAttitudeTempCalibData[4] = memsData.gxDS * 100000;
	sensorAttitudeTempCalibData[5] = memsData.gyDS * 100000;
	sensorAttitudeTempCalibData[6] = memsData.gzDS * 100000;
	sendConfigData(sensorAttitudeTempCalibData, 7, CMD_CALIBRATE_IMU_TEMP_DATA);
}

void calculateAccAndGyroTempCoeff() {
	float deltaTemp = 0;
	int16_t sampleCount = 0;
	uint8_t lpfInit = 0;
	float previousTemp = 0;
	float dt = SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY * 0.001;
	//Wait till the temperature is close to calibration temp
	do {
		delayMs(SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY);
		memsReadTemp();
		memsApplyTempDataScaling();
		if (!lpfInit) {
			lowPassFilterResetToValue(&sensorTempCalibLPF, memsData.tempC);
			lpfInit = 1;
		}
		memsData.tempC = lowPassFilterUpdate(&sensorTempCalibLPF, memsData.tempC, dt);
	} while (fabs(memsData.tempC - memsData.offsetTemp) > SESNSOR_TEMP_CAL_PROXIMITY_DEAD_BAND);
	previousTemp = memsData.tempC;
	lpfInit = 0;
	lowPassFilterResetToValue(&sensorTempCalibLPF, memsData.tempC);
	//Take measurements
	while (fabs(deltaTemp) <= SESNSOR_TEMP_CAL_RANGE) {
		delayMs(SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY);
		memsReadTemp();
		memsReadAccAndGyro();
		memsApplyAccOffsetCorrection();
		memsApplyGyroOffsetCorrection();
		memsApplyAccDataScaling();
		memsApplyGyroDataScaling();
		memsApplyTempDataScaling();
		if (!lpfInit) {
			lowPassFilterResetToValue(&sensorAccXCalibLPF, memsData.axG);
			lowPassFilterResetToValue(&sensorAccYCalibLPF, memsData.ayG);
			lowPassFilterResetToValue(&sensorAccZCalibLPF, memsData.azG);
			lowPassFilterResetToValue(&sensorGyroXCalibLPF, memsData.gxDS);
			lowPassFilterResetToValue(&sensorGyroYCalibLPF, memsData.gyDS);
			lowPassFilterResetToValue(&sensorGyroZCalibLPF, memsData.gzDS);
			lowPassFilterResetToValue(&sensorTempCalibLPF, memsData.tempC);
			lpfInit = 1;
		} else {
			memsData.axG = lowPassFilterUpdate(&sensorAccXCalibLPF, memsData.axG, dt);
			memsData.ayG = lowPassFilterUpdate(&sensorAccYCalibLPF, memsData.ayG, dt);
			memsData.azG = lowPassFilterUpdate(&sensorAccZCalibLPF, memsData.azG, dt);
			memsData.gxDS = lowPassFilterUpdate(&sensorGyroXCalibLPF, memsData.gxDS, dt);
			memsData.gyDS = lowPassFilterUpdate(&sensorGyroYCalibLPF, memsData.gyDS, dt);
			memsData.gzDS = lowPassFilterUpdate(&sensorGyroZCalibLPF, memsData.gzDS, dt);
			memsData.tempC = lowPassFilterUpdate(&sensorTempCalibLPF, memsData.tempC, dt);
		}
		deltaTemp = memsData.tempC - memsData.offsetTemp;
		if (fabs(previousTemp - memsData.tempC) >= SENSOR_TEMP_CAL_TEMP_DELTA) {
			previousTemp = memsData.tempC;
			memsData.tempC = deltaTemp;
			sendAttitudeTempCalibData();
		}
		sampleCount++;
		if (sampleCount == 50) {
			statusIndicatorToggle();
			sampleCount = 0;
		}
	}  //Delta while loop
	delayMs(SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY * 10);
}

