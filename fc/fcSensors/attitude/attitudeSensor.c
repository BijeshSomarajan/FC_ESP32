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

//BiQuald LPF and Notch Filters
BIQUADFILTER sensorAccXBiLpF, sensorAccYBiLpF, sensorAccZBiLpF;
BIQUADFILTER sensorGyroXBiLpF, sensorGyroYBiLpF, sensorGyroZBiLpF;
BIQUADFILTER sensorAccXBiNtF, sensorAccYBiNtF, sensorAccZBiNtF;
BIQUADFILTER sensorGyroXBiNtF, sensorGyroYBiNtF, sensorGyroZBiNtF;

//Low pass filters
LOWPASSFILTER sensorTempLPF;
LOWPASSFILTER sensorMagXLPF, sensorMagYLPF, sensorMagZLPF;

//Calibration related
LOWPASSFILTER sensorAccXCalibLPF, sensorAccYCalibLPF, sensorAccZCalibLPF;
LOWPASSFILTER sensorGyroXCalibLPF, sensorGyroYCalibLPF, sensorGyroZCalibLPF;
LOWPASSFILTER sensorTempCalibLPF;
int32_t sensorAttitudeTempCalibData[9];

float maxMotorNoiseFrequency;
float motorNoiseThFrequencyGain;
float currentGyroNoiseFrequency;
float currentAccNoiseFrequency;

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

uint8_t initAttitudeSensors(float motorKv, float batVolt, float nMotor) {
	uint8_t status = memsInit();
	if (status) {
		loadAttitudeSensorConfig();
		logString("[attitude] Calibration > Loaded\n");
		//Bi Quad LPF for ACC
		biQuadFilterInit(&sensorAccXBiLpF, BIQUAD_LOWPASS, SENSOR_ACC_BI_LPF_CENTER_FREQUENCY, SENSOR_ACC_SAMPLE_FREQUENCY, SENSOR_ACC_BI_LPF_Q, SENSOR_ACC_BI_LPF_PEAK_GAIN);
		biQuadFilterInit(&sensorAccYBiLpF, BIQUAD_LOWPASS, SENSOR_ACC_BI_LPF_CENTER_FREQUENCY, SENSOR_ACC_SAMPLE_FREQUENCY, SENSOR_ACC_BI_LPF_Q, SENSOR_ACC_BI_LPF_PEAK_GAIN);
		biQuadFilterInit(&sensorAccZBiLpF, BIQUAD_LOWPASS, SENSOR_ACC_BI_LPF_CENTER_FREQUENCY, SENSOR_ACC_SAMPLE_FREQUENCY, SENSOR_ACC_BI_LPF_Q, SENSOR_ACC_BI_LPF_PEAK_GAIN);
		//Bi Quad LPF for Gyro
		biQuadFilterInit(&sensorGyroXBiLpF, BIQUAD_LOWPASS, SENSOR_GYRO_BI_LPF_CENTER_FREQUENCY, SENSOR_GYRO_SAMPLE_FREQUENCY, SENSOR_GYRO_BI_LPF_Q, SENSOR_GYRO_BI_LPF_PEAK_GAIN);
		biQuadFilterInit(&sensorGyroYBiLpF, BIQUAD_LOWPASS, SENSOR_GYRO_BI_LPF_CENTER_FREQUENCY, SENSOR_GYRO_SAMPLE_FREQUENCY, SENSOR_GYRO_BI_LPF_Q, SENSOR_GYRO_BI_LPF_PEAK_GAIN);
		biQuadFilterInit(&sensorGyroZBiLpF, BIQUAD_LOWPASS, SENSOR_GYRO_BI_LPF_CENTER_FREQUENCY, SENSOR_GYRO_SAMPLE_FREQUENCY, SENSOR_GYRO_BI_LPF_Q, SENSOR_GYRO_BI_LPF_PEAK_GAIN);
		//LPF for Mag
		lowPassFilterInit(&sensorMagXLPF, SENSOR_MAG_LPF_FREQUENCY);
		lowPassFilterInit(&sensorMagYLPF, SENSOR_MAG_LPF_FREQUENCY);
		lowPassFilterInit(&sensorMagZLPF, SENSOR_MAG_LPF_FREQUENCY);
		//LPF for Temperature
		lowPassFilterInit(&sensorTempLPF, SENSOR_TEMP_LPF_FREQUENCY);

		//BiQuad NTF filters for ACC
		biQuadFilterInit(&sensorAccXBiNtF, BIQUAD_NOTCH, SENSOR_ACC_NTF_DEFAULT_CENTER_FREQ, SENSOR_ACC_SAMPLE_FREQUENCY, SENSOR_ACC_BI_NTF_Q, SENSOR_ACC_BI_NTF_PEAK_GAIN);
		biQuadFilterInit(&sensorAccYBiNtF, BIQUAD_NOTCH, SENSOR_ACC_NTF_DEFAULT_CENTER_FREQ, SENSOR_ACC_SAMPLE_FREQUENCY, SENSOR_ACC_BI_NTF_Q, SENSOR_ACC_BI_NTF_PEAK_GAIN);
		biQuadFilterInit(&sensorAccZBiNtF, BIQUAD_NOTCH, SENSOR_ACC_NTF_DEFAULT_CENTER_FREQ, SENSOR_ACC_SAMPLE_FREQUENCY, SENSOR_ACC_BI_NTF_Q, SENSOR_ACC_BI_NTF_PEAK_GAIN);
		//BiQuad NTF filters for Gyro
		biQuadFilterInit(&sensorGyroXBiNtF, BIQUAD_NOTCH, SENSOR_GYRO_NTF_DEFAULT_CENTER_FREQ, SENSOR_GYRO_SAMPLE_FREQUENCY, SENSOR_GYRO_BI_NTF_Q, SENSOR_GYRO_BI_NTF_PEAK_GAIN);
		biQuadFilterInit(&sensorGyroYBiNtF, BIQUAD_NOTCH, SENSOR_GYRO_NTF_DEFAULT_CENTER_FREQ, SENSOR_GYRO_SAMPLE_FREQUENCY, SENSOR_GYRO_BI_NTF_Q, SENSOR_GYRO_BI_NTF_PEAK_GAIN);
		biQuadFilterInit(&sensorGyroZBiNtF, BIQUAD_NOTCH, SENSOR_GYRO_NTF_DEFAULT_CENTER_FREQ, SENSOR_GYRO_SAMPLE_FREQUENCY, SENSOR_GYRO_BI_NTF_Q, SENSOR_GYRO_BI_NTF_PEAK_GAIN);

		//Low pass filters for Calibration
		lowPassFilterInit(&sensorAccXCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorAccYCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorAccZCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);

		lowPassFilterInit(&sensorGyroXCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorGyroYCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);
		lowPassFilterInit(&sensorGyroZCalibLPF, SENSOR_AG_CALIB_LOWPASS_FREQ);

		lowPassFilterInit(&sensorTempCalibLPF,  SENSOR_AG_TEMP_CALIB_LOWPASS_FREQ);

		logString("[attitude] > Filters initialized Success\n");

		maxMotorNoiseFrequency = ((motorKv * batVolt) / 60.0f) * nMotor;
		motorNoiseThFrequencyGain = (maxMotorNoiseFrequency / 100.0f) * SENSOR_ACC_GYRO_NOISE_GAIN_FACTOR;
		calculateMotorNoise(0);

		logString("[attitude] > Success\n");
	} else {
		logString("[attitude] > Failed\n");
	}
	return status;
}

void calculateMotorNoise(float throttlePercentage) {
	currentGyroNoiseFrequency = maxMotorNoiseFrequency * (throttlePercentage <= 0 ? 1 : throttlePercentage) * motorNoiseThFrequencyGain;
	currentAccNoiseFrequency = currentGyroNoiseFrequency;

	if (currentAccNoiseFrequency < SENSOR_ACC_NTF_DEFAULT_CENTER_FREQ) {
		currentAccNoiseFrequency = SENSOR_ACC_NTF_DEFAULT_CENTER_FREQ;
	}
	if (currentGyroNoiseFrequency < SENSOR_GYRO_NTF_DEFAULT_CENTER_FREQ) {
		currentGyroNoiseFrequency = SENSOR_GYRO_NTF_DEFAULT_CENTER_FREQ;
	}
	biQuadFilterSetCenterFreq(&sensorAccXBiNtF, currentAccNoiseFrequency);
	biQuadFilterSetCenterFreq(&sensorAccYBiNtF, currentAccNoiseFrequency);
	biQuadFilterSetCenterFreq(&sensorAccZBiNtF, currentAccNoiseFrequency);
	biQuadFilterSetCenterFreq(&sensorGyroXBiNtF, currentGyroNoiseFrequency);
	biQuadFilterSetCenterFreq(&sensorGyroYBiNtF, currentGyroNoiseFrequency);
	biQuadFilterSetCenterFreq(&sensorGyroZBiNtF, currentGyroNoiseFrequency);
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

	//Apply notch filtering
	attitudeData.axG = biQuadFilterUpdate(&sensorAccXBiNtF, attitudeData.axG);
	attitudeData.ayG = biQuadFilterUpdate(&sensorAccYBiNtF, attitudeData.ayG);
	attitudeData.azG = biQuadFilterUpdate(&sensorAccZBiNtF, attitudeData.azG);

	//Apply lowpass filtering
	attitudeData.axG = biQuadFilterUpdate(&sensorAccXBiLpF, attitudeData.axG);
	attitudeData.ayG = biQuadFilterUpdate(&sensorAccYBiLpF, attitudeData.ayG);
	attitudeData.azG = biQuadFilterUpdate(&sensorAccZBiLpF, attitudeData.azG);

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

	//Apply Notch Filtering
	attitudeData.gxDS = biQuadFilterUpdate(&sensorGyroXBiNtF, attitudeData.gxDS);
	attitudeData.gyDS = biQuadFilterUpdate(&sensorGyroYBiNtF, attitudeData.gyDS);
	attitudeData.gzDS = biQuadFilterUpdate(&sensorGyroZBiNtF, attitudeData.gzDS);

	//Bi Quad LPF Filtering
	attitudeData.gxDS = biQuadFilterUpdate(&sensorGyroXBiLpF, attitudeData.gxDS);
	attitudeData.gyDS = biQuadFilterUpdate(&sensorGyroYBiLpF, attitudeData.gyDS);
	attitudeData.gzDS = biQuadFilterUpdate(&sensorGyroZBiLpF, attitudeData.gzDS);
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
void applyAGTempCorrection(float currentTemp) {
#if SENSOR_APPLY_ACC_TEMP_OFFSET_CORRECTION ==1 || SENSOR_APPLY_GYRO_TEMP_OFFSET_CORRECTION ==1
	//temp_bias[0] = gyro_coeff_x[0] + gyro_coeff_x[1] * t +  gyro_coeff_x[2] * powf(t,2) + gyro_coeff_x[3] * powf(t,3);
	float tempP1 = (currentTemp - memsData.offsetTemp);
	float tempP2 = tempP1 * tempP1;
	float tempP3 = tempP2 * tempP1;
#if SENSOR_APPLY_ACC_TEMP_OFFSET_CORRECTION ==1
	//Acclerometer data is observed to be already temp compensated
	memsData.accXTempOffset = memsData.accXTempCoeff[0] + memsData.accXTempCoeff[1] * tempP1 + memsData.accXTempCoeff[2] * tempP2 + memsData.accXTempCoeff[3] * tempP3;
	memsData.accYTempOffset = memsData.accYTempCoeff[0] + memsData.accYTempCoeff[1] * tempP1 + memsData.accYTempCoeff[2] * tempP2 + memsData.accYTempCoeff[3] * tempP3;
	memsData.accZTempOffset = memsData.accZTempCoeff[0] + memsData.accZTempCoeff[1] * tempP1 + memsData.accZTempCoeff[2] * tempP2 + memsData.accZTempCoeff[3] * tempP3;
#endif
#if SENSOR_APPLY_GYRO_TEMP_OFFSET_CORRECTION ==1
	memsData.gyroXTempOffset = memsData.gyroXTempCoeff[0] + memsData.gyroXTempCoeff[1] * tempP1 + memsData.gyroXTempCoeff[2] * tempP2 + memsData.gyroXTempCoeff[3] * tempP3;
	memsData.gyroYTempOffset = memsData.gyroYTempCoeff[0] + memsData.gyroYTempCoeff[1] * tempP1 + memsData.gyroYTempCoeff[2] * tempP2 + memsData.gyroYTempCoeff[3] * tempP3;
	memsData.gyroZTempOffset = memsData.gyroZTempCoeff[0] + memsData.gyroZTempCoeff[1] * tempP1 + memsData.gyroZTempCoeff[2] * tempP2 + memsData.gyroZTempCoeff[3] * tempP3;
#endif
#endif
}

void readTempSensor(float dt) {
	memsReadTemp();
	memsApplyTempDataScaling();

	lowPassFilterUpdate(&sensorTempLPF, memsData.tempC, dt);
	attitudeData.temp = sensorTempLPF.output;
	applyAGTempCorrection(attitudeData.temp);
}

void readMagSensor(float dt) {
	memsReadMag();
	memsApplyMagDataScaling();
	memsApplyMagOffsetCorrection();
	//Aligning the axis to Gyro and Acc
	lowPassFilterUpdate(&sensorMagXLPF, -memsData.mx, dt);
	lowPassFilterUpdate(&sensorMagYLPF, memsData.my, dt);
	lowPassFilterUpdate(&sensorMagZLPF, memsData.mz, dt);
	//Assign the smoothened values for further use
	attitudeData.mx = sensorMagXLPF.output;
	attitudeData.my = sensorMagYLPF.output;
	attitudeData.mz = sensorMagZLPF.output;
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

