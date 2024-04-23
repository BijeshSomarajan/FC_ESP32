#include "attitudeSensor.h"
#include "deltaTimer.h"
#include "calibration.h"
#include "biQuadFilter.h"
#include "lowPassFilter.h"
#include "indicator.h"
#include "configSensor.h"
#include "delayTimer.h"

ATTITUDE_DATA attitudeData;

BIQUADFILTER sensorAccXBiLpF, sensorAccYBiLpF, sensorAccZBiLpF;
BIQUADFILTER sensorGyroXBiLpF, sensorGyroYBiLpF, sensorGyroZBiLpF;
LOWPASSFILTER sensorTempLPF;
LOWPASSFILTER sensorMagXLPF, sensorMagYLPF, sensorMagZLPF;

//Calibration related
LOWPASSFILTER sensorAccXCalibLPF, sensorAccYCalibLPF, sensorAccZCalibLPF;
LOWPASSFILTER sensorGyroXCalibLPF, sensorGyroYCalibLPF, sendorGyroZCalibLPF;
LOWPASSFILTER lsm9ds1TempCalibLPF;
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
	//Reset ACC BiQuad filter
	biQuadFilterReset(&sensorAccXBiLpF);
	biQuadFilterReset(&sensorAccYBiLpF);
	biQuadFilterReset(&sensorAccZBiLpF);
	//Reset Gyro BiQuad filter
	biQuadFilterReset(&sensorGyroXBiLpF);
	biQuadFilterReset(&sensorGyroYBiLpF);
	biQuadFilterReset(&sensorGyroZBiLpF);
	//Reset Mag LPF filter
	lowPassFilterReset(&sensorMagXLPF);
	lowPassFilterReset(&sensorMagYLPF);
	lowPassFilterReset(&sensorMagZLPF);
	//Reset Temp LPF filter
	lowPassFilterReset(&sensorTempLPF);
}

void loadAttitudeSensorConfig() {
	lsm9ds1.offsetAx = getCalibrationValue(CALIB_PROP_AX_BIAS_ADDR);
	lsm9ds1.offsetAy = getCalibrationValue(CALIB_PROP_AY_BIAS_ADDR);
	lsm9ds1.offsetAz = getCalibrationValue(CALIB_PROP_AZ_BIAS_ADDR);

	lsm9ds1.offsetGx = getCalibrationValue(CALIB_PROP_GX_BIAS_ADDR);
	lsm9ds1.offsetGy = getCalibrationValue(CALIB_PROP_GY_BIAS_ADDR);
	lsm9ds1.offsetGz = getCalibrationValue(CALIB_PROP_GZ_BIAS_ADDR);

	lsm9ds1.offsetMx = getScaledCalibrationValue(CALIB_PROP_MX_OFFSET_ADDR);
	lsm9ds1.offsetMy = getScaledCalibrationValue(CALIB_PROP_MY_OFFSET_ADDR);
	lsm9ds1.offsetMz = getScaledCalibrationValue(CALIB_PROP_MZ_OFFSET_ADDR);

	lsm9ds1.biasMx = getScaledCalibrationValue(CALIB_PROP_MX_BIAS_ADDR);
	lsm9ds1.biasMy = getScaledCalibrationValue(CALIB_PROP_MY_BIAS_ADDR);
	lsm9ds1.biasMz = getScaledCalibrationValue(CALIB_PROP_MZ_BIAS_ADDR);

	lsm9ds1.scaleMx = getScaledCalibrationValue(CALIB_PROP_MX_SCALE_ADDR);
	lsm9ds1.scaleMy = getScaledCalibrationValue(CALIB_PROP_MY_SCALE_ADDR);
	lsm9ds1.scaleMz = getScaledCalibrationValue(CALIB_PROP_MZ_SCALE_ADDR);

	lsm9ds1.offsetTemp = getScaledCalibrationValue(CALIB_PROP_IMU_TEMP_ADDR);
	lsm9ds1.tempC = lsm9ds1.offsetTemp;

	//Higher order coefficients are divided by higher powers of 10
	lsm9ds1.accXTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C_ADDR) / 10000.0f; //3136.34291;
	lsm9ds1.accXTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C1_ADDR) / 100000.0f; //-66.95038;
	lsm9ds1.accXTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C2_ADDR) / 1000000.0f; //-3.85809;
	lsm9ds1.accXTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AX_C3_ADDR) / 10000000.0f; //0.04168;

	lsm9ds1.accYTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C_ADDR) / 10000.0f; //3099.9199;
	lsm9ds1.accYTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C1_ADDR) / 100000.0f; //-111.3936;
	lsm9ds1.accYTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C2_ADDR) / 1000000.0f; //-1.23488;
	lsm9ds1.accYTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AY_C3_ADDR) / 10000000.0f; //0.00965;

	lsm9ds1.accZTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C_ADDR) / 10000.0f; //-14301.91832;
	lsm9ds1.accZTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C1_ADDR) / 100000.0f; //6.38;
	lsm9ds1.accZTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C2_ADDR) / 1000000.0f; //-4.84;
	lsm9ds1.accZTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_AZ_C3_ADDR) / 10000000.0f; //0.03351;

	lsm9ds1.gyroXTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C_ADDR) / 10000.0f; //12.8099;
	lsm9ds1.gyroXTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C1_ADDR) / 100000.0f; //-1.85993;
	lsm9ds1.gyroXTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C2_ADDR) / 1000000.0f; //0.4426;
	lsm9ds1.gyroXTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GX_C3_ADDR) / 10000000.0f; //-0.00890;

	lsm9ds1.gyroYTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C_ADDR) / 10000.0f; //10.94848;
	lsm9ds1.gyroYTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C1_ADDR) / 100000.0f; //-.46218;
	lsm9ds1.gyroYTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C2_ADDR) / 1000000.0f; //0.20199;
	lsm9ds1.gyroYTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GY_C3_ADDR) / 10000000.0f; //-0.002;

	lsm9ds1.gyroZTempCoeff[0] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C_ADDR) / 10000.0f; //181.0141
	lsm9ds1.gyroZTempCoeff[1] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C1_ADDR) / 100000.0f; //-6.10440;
	lsm9ds1.gyroZTempCoeff[2] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C2_ADDR) / 1000000.0f; //-0.608;
	lsm9ds1.gyroZTempCoeff[3] = getCalibrationValue(CALIB_PROP_IMU_TEMP_COEFF_GZ_C3_ADDR) / 10000000.0f; //0.00962;
}

uint8_t initAttitudeSensors() {
	uint8_t status = initlsm9ds1();
	if (status) {
		loadAttitudeSensorConfig();
		logString("[attitude] Calibration > Loaded\n");
		//Bi Quad LPF for ACC
		biQuadFilterInit(&sensorAccXBiLpF, BIQUAD_LOWPASS, SENSOR_ACC_BI_LPF_CENTER_FREQUENCY, LSM9DS1_ACC_SAMPLE_FREQUENCY, SENSOR_ACC_BI_LPF_Q, SENSOR_ACC_BI_LPF_PEAK_GAIN);
		biQuadFilterInit(&sensorAccYBiLpF, BIQUAD_LOWPASS, SENSOR_ACC_BI_LPF_CENTER_FREQUENCY, LSM9DS1_ACC_SAMPLE_FREQUENCY, SENSOR_ACC_BI_LPF_Q, SENSOR_ACC_BI_LPF_PEAK_GAIN);
		biQuadFilterInit(&sensorAccZBiLpF, BIQUAD_LOWPASS, SENSOR_ACC_BI_LPF_CENTER_FREQUENCY, LSM9DS1_ACC_SAMPLE_FREQUENCY, SENSOR_ACC_BI_LPF_Q, SENSOR_ACC_BI_LPF_PEAK_GAIN);
		//Bi Quad LPF for Gyro
		biQuadFilterInit(&sensorGyroXBiLpF, BIQUAD_LOWPASS, SENSOR_GYRO_BI_LPF_CENTER_FREQUENCY, LSM9DS1_GYRO_SAMPLE_FREQUENCY, SENSOR_GYRO_BI_LPF_Q, SENSOR_GYRO_BI_LPF_PEAK_GAIN);
		biQuadFilterInit(&sensorGyroYBiLpF, BIQUAD_LOWPASS, SENSOR_GYRO_BI_LPF_CENTER_FREQUENCY, LSM9DS1_GYRO_SAMPLE_FREQUENCY, SENSOR_GYRO_BI_LPF_Q, SENSOR_GYRO_BI_LPF_PEAK_GAIN);
		biQuadFilterInit(&sensorGyroZBiLpF, BIQUAD_LOWPASS, SENSOR_GYRO_BI_LPF_CENTER_FREQUENCY, LSM9DS1_GYRO_SAMPLE_FREQUENCY, SENSOR_GYRO_BI_LPF_Q, SENSOR_GYRO_BI_LPF_PEAK_GAIN);
		//LPF for Mag
		lowPassFilterInit(&sensorMagXLPF, SENSOR_MAG_LPF_FREQUENCY);
		lowPassFilterInit(&sensorMagYLPF, SENSOR_MAG_LPF_FREQUENCY);
		lowPassFilterInit(&sensorMagZLPF, SENSOR_MAG_LPF_FREQUENCY);
		//LPF for Temperature
		lowPassFilterInit(&sensorTempLPF, SENSOR_TEMP_LPF_FREQUENCY);

		//Low pass filters for Calibration
		lowPassFilterInit(&sensorAccXCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorAccYCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorAccZCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);

		lowPassFilterInit(&sensorGyroXCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorGyroYCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sendorGyroZCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);

		lowPassFilterInit(&lsm9ds1TempCalibLPF, LSM9DS1_TEMP_CALIB_LOWPASS_FREQ);

		logString("[attitude] > Filters initialized Success\n");

		logString("[attitude] > Success\n");
	} else {
		logString("[attitude] > Failed\n");
	}
	return status;
}

void processAccSensorData(float dt) {
	lsm9ds1ApplyAccOffsetCorrection();
	lsm9ds1ApplyAccDataScaling();
	lsm9ds1ApplyAccTempOffsetCorrection();

	attitudeData.axGRaw = lsm9ds1.axG;
	attitudeData.ayGRaw = lsm9ds1.ayG;
	attitudeData.azGRaw = (1 + lsm9ds1.azG);

	attitudeData.axG = lsm9ds1.axG;
	attitudeData.ayG = lsm9ds1.ayG;
	attitudeData.azG = (1 + lsm9ds1.azG);

	//Limit the values
	attitudeData.axG = constrainToRangeF(attitudeData.axG, -SENSOR_ACC_FLYABLE_VALUE_XY_LIMIT, SENSOR_ACC_FLYABLE_VALUE_XY_LIMIT);
	attitudeData.ayG = constrainToRangeF(attitudeData.ayG, -SENSOR_ACC_FLYABLE_VALUE_XY_LIMIT, SENSOR_ACC_FLYABLE_VALUE_XY_LIMIT);
	attitudeData.azG = constrainToRangeF(attitudeData.azG, -SENSOR_ACC_FLYABLE_VALUE_Z_LIMIT, SENSOR_ACC_FLYABLE_VALUE_Z_LIMIT);

	attitudeData.axG = biQuadFilterUpdate(&sensorAccXBiLpF, attitudeData.axG);
	attitudeData.ayG = biQuadFilterUpdate(&sensorAccYBiLpF, attitudeData.ayG);
	attitudeData.azG = biQuadFilterUpdate(&sensorAccZBiLpF, attitudeData.azG);
}

void processGyroSensorData(float dt) {
	lsm9ds1ApplyGyroOffsetCorrection();
	lsm9ds1ApplyGyroDataScaling();
	lsm9ds1ApplyGyroTempOffsetCorrection();

	attitudeData.gxDSRaw = lsm9ds1.gxDS;
	attitudeData.gyDSRaw = lsm9ds1.gyDS;
	attitudeData.gzDSRaw = lsm9ds1.gzDS;

	attitudeData.gxDS = -lsm9ds1.gxDS;
	attitudeData.gyDS = -lsm9ds1.gyDS;
	attitudeData.gzDS = -lsm9ds1.gzDS;

	//Limit the values
	attitudeData.gxDS = constrainToRangeF(attitudeData.gxDS, -SENSOR_GYRO_FLYABLE_VALUE_LIMIT, SENSOR_GYRO_FLYABLE_VALUE_LIMIT);
	attitudeData.gyDS = constrainToRangeF(attitudeData.gyDS, -SENSOR_GYRO_FLYABLE_VALUE_LIMIT, SENSOR_GYRO_FLYABLE_VALUE_LIMIT);
	attitudeData.gzDS = constrainToRangeF(attitudeData.gzDS, -SENSOR_GYRO_FLYABLE_VALUE_LIMIT, SENSOR_GYRO_FLYABLE_VALUE_LIMIT);

	//Bi Quad LPF Filtering
	attitudeData.gxDS = biQuadFilterUpdate(&sensorGyroXBiLpF, attitudeData.gxDS);
	attitudeData.gyDS = biQuadFilterUpdate(&sensorGyroYBiLpF, attitudeData.gyDS);
	attitudeData.gzDS = biQuadFilterUpdate(&sensorGyroZBiLpF, attitudeData.gzDS);
}

void readAccSensor(float dt) {
	lsm9ds1ReadAcc();
	processAccSensorData(dt);
}

void readGyroSensor(float dt) {
	lsm9ds1ReadGyro();
	processGyroSensorData(dt);
}

void readAccAndGyroSensor(float dt) {
	lsm9ds1ReadAccAndGyro();
	processAccSensorData(dt);
	processGyroSensorData(dt);
}

/**
 Calculates the temperature offsets by 3rd polynomial
 **/
void applyAGTempCorrection(float currentTemp) {
#if LSM9DS1_APPLY_ACC_TEMP_OFFSET_CORRECTION ==1 || LSM9DS1_APPLY_GYRO_TEMP_OFFSET_CORRECTION ==1
	//temp_bias[0] = gyro_coeff_x[0] + gyro_coeff_x[1] * t +  gyro_coeff_x[2] * powf(t,2) + gyro_coeff_x[3] * powf(t,3);
	float tempP1 = (currentTemp - lsm9ds1.offsetTemp);
	float tempP2 = tempP1 * tempP1;
	float tempP3 = tempP2 * tempP1;
#if LSM9DS1_APPLY_ACC_TEMP_OFFSET_CORRECTION ==1
	//Acclerometer data is observed to be already temp compensated
	lsm9ds1.accXTempOffset = lsm9ds1.accXTempCoeff[0] + lsm9ds1.accXTempCoeff[1] * tempP1 + lsm9ds1.accXTempCoeff[2] * tempP2 + lsm9ds1.accXTempCoeff[3] * tempP3;
	lsm9ds1.accYTempOffset = lsm9ds1.accYTempCoeff[0] + lsm9ds1.accYTempCoeff[1] * tempP1 + lsm9ds1.accYTempCoeff[2] * tempP2 + lsm9ds1.accYTempCoeff[3] * tempP3;
	lsm9ds1.accZTempOffset = lsm9ds1.accZTempCoeff[0] + lsm9ds1.accZTempCoeff[1] * tempP1 + lsm9ds1.accZTempCoeff[2] * tempP2 + lsm9ds1.accZTempCoeff[3] * tempP3;
#endif
#if LSM9DS1_APPLY_GYRO_TEMP_OFFSET_CORRECTION ==1
	lsm9ds1.gyroXTempOffset = lsm9ds1.gyroXTempCoeff[0] + lsm9ds1.gyroXTempCoeff[1] * tempP1 + lsm9ds1.gyroXTempCoeff[2] * tempP2 + lsm9ds1.gyroXTempCoeff[3] * tempP3;
	lsm9ds1.gyroYTempOffset = lsm9ds1.gyroYTempCoeff[0] + lsm9ds1.gyroYTempCoeff[1] * tempP1 + lsm9ds1.gyroYTempCoeff[2] * tempP2 + lsm9ds1.gyroYTempCoeff[3] * tempP3;
	lsm9ds1.gyroZTempOffset = lsm9ds1.gyroZTempCoeff[0] + lsm9ds1.gyroZTempCoeff[1] * tempP1 + lsm9ds1.gyroZTempCoeff[2] * tempP2 + lsm9ds1.gyroZTempCoeff[3] * tempP3;
#endif
#endif
}
void readTempSensor(float dt) {
	lsm9ds1ReadTemp();
	lowPassFilterUpdate(&sensorTempLPF, lsm9ds1.tempC, dt);
	attitudeData.temp = sensorTempLPF.output;
	applyAGTempCorrection(attitudeData.temp);
}
void readMagSensor(float dt) {
	lsm9ds1ReadMag();
	lsm9ds1ApplyMagDataScaling();
	lsm9ds1ApplyMagOffsetCorrection();
	//Aligning the axis to Gyro and Acc
	lowPassFilterUpdate(&sensorMagXLPF, -lsm9ds1.mx, dt);
	lowPassFilterUpdate(&sensorMagYLPF, lsm9ds1.my, dt);
	lowPassFilterUpdate(&sensorMagZLPF, lsm9ds1.mz, dt);
	//Assign the smoothened values for further use
	attitudeData.mx = sensorMagXLPF.output;
	attitudeData.my = sensorMagYLPF.output;
	attitudeData.mz = sensorMagZLPF.output;
}

void calculateAccAndGyroBias() {
	//Calibrate lsm9ds1 Acc and Gyro
	lsm9ds1.offsetAx = 0;
	lsm9ds1.offsetAy = 0;
	lsm9ds1.offsetAz = 0;
	lsm9ds1.offsetGx = 0;
	lsm9ds1.offsetGy = 0;
	lsm9ds1.offsetGz = 0;
	uint8_t lpfInit = 0;
	//Take average of Acc and Gyro readings
	for (int16_t sampleCount = 0; sampleCount < SENSOR_AG_OFFSET_CALIB_SAMPLE_COUNT; sampleCount++) {
		lsm9ds1ReadAccAndGyro();
		lsm9ds1ReadTemp();
		if (!lpfInit) {
			lowPassFilterResetToValue(&sensorAccXCalibLPF, lsm9ds1.rawAx);
			lowPassFilterResetToValue(&sensorAccYCalibLPF, lsm9ds1.rawAy);
			lowPassFilterResetToValue(&sensorAccZCalibLPF, lsm9ds1.rawAz);
			lowPassFilterResetToValue(&sensorGyroXCalibLPF, lsm9ds1.rawGx);
			lowPassFilterResetToValue(&sensorGyroYCalibLPF, lsm9ds1.rawGy);
			lowPassFilterResetToValue(&sendorGyroZCalibLPF, lsm9ds1.rawGz);
			lowPassFilterResetToValue(&lsm9ds1TempCalibLPF, lsm9ds1.tempC);
			lpfInit = 1;
		}
		float dt = SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY * 0.001;
		lsm9ds1.offsetAx = lowPassFilterUpdate(&sensorAccXCalibLPF, lsm9ds1.rawAx, dt);
		lsm9ds1.offsetAy = lowPassFilterUpdate(&sensorAccYCalibLPF, lsm9ds1.rawAy, dt);
		lsm9ds1.offsetAz = lowPassFilterUpdate(&sensorAccZCalibLPF, lsm9ds1.rawAz, dt);
		lsm9ds1.offsetGx = lowPassFilterUpdate(&sensorGyroXCalibLPF, lsm9ds1.rawGx, dt);
		lsm9ds1.offsetGy = lowPassFilterUpdate(&sensorGyroYCalibLPF, lsm9ds1.rawGy, dt);
		lsm9ds1.offsetGz = lowPassFilterUpdate(&sendorGyroZCalibLPF, lsm9ds1.rawGz, dt);
		lsm9ds1.offsetTemp = lowPassFilterUpdate(&lsm9ds1TempCalibLPF, lsm9ds1.tempC, dt);
		delayMs(SENSOR_AG_OFFSET_CALIB_SAMPLE_DELAY);
		if (sampleCount % 10 == 0) {
			statusIndicatorToggle();
		}
	}

	//Back fill data for persistence
	setCalibrationValue(CALIB_PROP_AX_BIAS_ADDR, lsm9ds1.offsetAx);
	setCalibrationValue(CALIB_PROP_AY_BIAS_ADDR, lsm9ds1.offsetAy);
	setCalibrationValue(CALIB_PROP_AZ_BIAS_ADDR, lsm9ds1.offsetAz);
	setCalibrationValue(CALIB_PROP_GX_BIAS_ADDR, lsm9ds1.offsetGx);
	setCalibrationValue(CALIB_PROP_GY_BIAS_ADDR, lsm9ds1.offsetGy);
	setCalibrationValue(CALIB_PROP_GZ_BIAS_ADDR, lsm9ds1.offsetGz);
	setCalibrationValue(CALIB_PROP_IMU_TEMP_ADDR, getCalibrationScalableValue(lsm9ds1.offsetTemp));
	//Persist the calibration
	saveCalibration();
}

void calculateMagBias() {
	lsm9ds1.offsetMx = 0;
	lsm9ds1.offsetMy = 0;
	lsm9ds1.offsetMz = 0;
	//Determining magnetometer bias , Move the device in 8 pattern
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 };
	for (int indx = 0; indx < SENSOR_MAG_CALIB_SAMPLE_COUNT; indx++) {
		// Read the mag data
		lsm9ds1ReadMag();

		if (lsm9ds1.rawMx > mag_max[0]) {
			mag_max[0] = lsm9ds1.rawMx;
		} else if (lsm9ds1.rawMx < mag_min[0]) {
			mag_min[0] = lsm9ds1.rawMx;
		}

		if (lsm9ds1.rawMy > mag_max[1]) {
			mag_max[1] = lsm9ds1.rawMy;
		} else if (lsm9ds1.rawMy < mag_min[1]) {
			mag_min[1] = lsm9ds1.rawMy;
		}

		if (lsm9ds1.rawMz > mag_max[2]) {
			mag_max[2] = lsm9ds1.rawMz;
		} else if (lsm9ds1.rawMz < mag_min[2]) {
			mag_min[2] = lsm9ds1.rawMz;
		}

		delayMs(SENSOR_MAG_CALIB_SAMPLE_DELAY);
		statusIndicatorToggle();
	}

	// Get hard iron correction , Bias
	lsm9ds1.biasMx = ((float) (mag_max[0] + mag_min[0]) / 2.0f) * lsm9ds1.magSensitivity;
	lsm9ds1.biasMy = ((float) (mag_max[1] + mag_min[1]) / 2.0f) * lsm9ds1.magSensitivity;
	lsm9ds1.biasMz = ((float) (mag_max[2] + mag_min[2]) / 2.0f) * lsm9ds1.magSensitivity;

	// Get soft iron correction estimate
	lsm9ds1.scaleMx = ((float) (mag_max[0] - mag_min[0])) / 2.0f; // get average x axis max chord length in counts
	lsm9ds1.scaleMy = ((float) (mag_max[1] - mag_min[1])) / 2.0f; // get average y axis max chord length in counts
	lsm9ds1.scaleMz = ((float) (mag_max[2] - mag_min[2])) / 2.0f; // get average z axis max chord length in counts

	float avg_rad = (lsm9ds1.scaleMx + lsm9ds1.scaleMy + lsm9ds1.scaleMz) / 3.0f;

	lsm9ds1.scaleMx = avg_rad / lsm9ds1.scaleMx;
	lsm9ds1.scaleMy = avg_rad / lsm9ds1.scaleMy;
	lsm9ds1.scaleMz = avg_rad / lsm9ds1.scaleMz;
	//Back fill data for persistence
	setCalibrationValue(CALIB_PROP_MX_OFFSET_ADDR, getCalibrationScalableValue(lsm9ds1.offsetMx));
	setCalibrationValue(CALIB_PROP_MY_OFFSET_ADDR, getCalibrationScalableValue(lsm9ds1.offsetMy));
	setCalibrationValue(CALIB_PROP_MZ_OFFSET_ADDR, getCalibrationScalableValue(lsm9ds1.offsetMz));

	setCalibrationValue(CALIB_PROP_MX_BIAS_ADDR, getCalibrationScalableValue(lsm9ds1.biasMx));
	setCalibrationValue(CALIB_PROP_MY_BIAS_ADDR, getCalibrationScalableValue(lsm9ds1.biasMy));
	setCalibrationValue(CALIB_PROP_MZ_BIAS_ADDR, getCalibrationScalableValue(lsm9ds1.biasMz));

	setCalibrationValue(CALIB_PROP_MX_SCALE_ADDR, getCalibrationScalableValue(lsm9ds1.scaleMx));
	setCalibrationValue(CALIB_PROP_MY_SCALE_ADDR, getCalibrationScalableValue(lsm9ds1.scaleMy));
	setCalibrationValue(CALIB_PROP_MZ_SCALE_ADDR, getCalibrationScalableValue(lsm9ds1.scaleMz));

	//Persist the calibration
	saveCalibration();
}

/***********************************************************************/
/* Send the calibration data                                           */
/***********************************************************************/
void sendAttitudeTempCalibData() {
	sensorAttitudeTempCalibData[0] = lsm9ds1.tempC * 100000;
	sensorAttitudeTempCalibData[1] = lsm9ds1.axG * 100000;
	sensorAttitudeTempCalibData[2] = lsm9ds1.ayG * 100000;
	sensorAttitudeTempCalibData[3] = lsm9ds1.azG * 100000;
	sensorAttitudeTempCalibData[4] = lsm9ds1.gxDS * 100000;
	sensorAttitudeTempCalibData[5] = lsm9ds1.gyDS * 100000;
	sensorAttitudeTempCalibData[6] = lsm9ds1.gzDS * 100000;
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
		lsm9ds1ReadTemp();
		if (!lpfInit) {
			lowPassFilterResetToValue(&lsm9ds1TempCalibLPF, lsm9ds1.tempC);
			lpfInit = 1;
		}
		lsm9ds1.tempC = lowPassFilterUpdate(&lsm9ds1TempCalibLPF, lsm9ds1.tempC, dt);
	} while (fabs(lsm9ds1.tempC - lsm9ds1.offsetTemp) > LSM9DS1_TEMP_CAL_PROXIMITY_DEAD_BAND);
	previousTemp = lsm9ds1.tempC;
	lpfInit = 0;
	lowPassFilterResetToValue(&lsm9ds1TempCalibLPF, lsm9ds1.tempC);
	//Take measurements
	while (fabs(deltaTemp) <= SESNSOR_TEMP_CAL_RANGE) {
		delayMs(SENSOR_AG_TEMP_CALIB_SAMPLE_DELAY);
		lsm9ds1ReadTemp();
		lsm9ds1ReadAccAndGyro();
		lsm9ds1ApplyAccOffsetCorrection();
		lsm9ds1ApplyGyroOffsetCorrection();
		lsm9ds1ApplyAccDataScaling();
		lsm9ds1ApplyGyroDataScaling();
		if (!lpfInit) {
			lowPassFilterResetToValue(&sensorAccXCalibLPF, lsm9ds1.axG);
			lowPassFilterResetToValue(&sensorAccYCalibLPF, lsm9ds1.ayG);
			lowPassFilterResetToValue(&sensorAccZCalibLPF, lsm9ds1.azG);
			lowPassFilterResetToValue(&sensorGyroXCalibLPF, lsm9ds1.gxDS);
			lowPassFilterResetToValue(&sensorGyroYCalibLPF, lsm9ds1.gyDS);
			lowPassFilterResetToValue(&sendorGyroZCalibLPF, lsm9ds1.gzDS);
			lowPassFilterResetToValue(&lsm9ds1TempCalibLPF, lsm9ds1.tempC);
			lpfInit = 1;
		} else {
			lsm9ds1.axG = lowPassFilterUpdate(&sensorAccXCalibLPF, lsm9ds1.axG, dt);
			lsm9ds1.ayG = lowPassFilterUpdate(&sensorAccYCalibLPF, lsm9ds1.ayG, dt);
			lsm9ds1.azG = lowPassFilterUpdate(&sensorAccZCalibLPF, lsm9ds1.azG, dt);
			lsm9ds1.gxDS = lowPassFilterUpdate(&sensorGyroXCalibLPF, lsm9ds1.gxDS, dt);
			lsm9ds1.gyDS = lowPassFilterUpdate(&sensorGyroYCalibLPF, lsm9ds1.gyDS, dt);
			lsm9ds1.gzDS = lowPassFilterUpdate(&sendorGyroZCalibLPF, lsm9ds1.gzDS, dt);
			lsm9ds1.tempC = lowPassFilterUpdate(&lsm9ds1TempCalibLPF, lsm9ds1.tempC, dt);
		}
		deltaTemp = lsm9ds1.tempC - lsm9ds1.offsetTemp;
		if (fabs(previousTemp - lsm9ds1.tempC) >= SENSOR_TEMP_CAL_TEMP_DELTA) {
			previousTemp = lsm9ds1.tempC;
			lsm9ds1.tempC = deltaTemp;
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
