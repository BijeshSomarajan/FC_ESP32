#include "noiseSensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "deltaTimer.h"
#include "fcLogger.h"
#include "attitudeSensor.h"
#include "biQuadFilter.h"
#include "fcStatus.h"

FFTContext fftX;
FFTContext fftY;
FFTContext fftZ;

//BiQuald LPF and Notch Filters
LOWPASSFILTER noiseSensorAccXLpF, noiseSensorAccYLpF, noiseSensorAccZLpF;
LOWPASSFILTER noiseSensorGyroXLpF, noiseSensorGyroYLpF, noiseSensorGyroZLpF;

const float NOISE_SENSOR_NTF_ACC_Q_ARRAY[3][3] = NOISE_SENSOR_NTF_ACC_Q;
const float NOISE_SENSOR_NTF_GYRO_Q_ARRAY[3][3] = NOISE_SENSOR_NTF_GYRO_Q;

BIQUADFILTER noiseSensorAccXBiNTF[NOISE_SENSOR_NTF_ACC_FREQUENCY_N][NOISE_SENSOR_NTF_GYRO_HARMONIC_N];
BIQUADFILTER noiseSensorAccYBiNTF[NOISE_SENSOR_NTF_ACC_FREQUENCY_N][NOISE_SENSOR_NTF_GYRO_HARMONIC_N];
BIQUADFILTER noiseSensorAccZBiNTF[NOISE_SENSOR_NTF_ACC_FREQUENCY_N][NOISE_SENSOR_NTF_GYRO_HARMONIC_N];

BIQUADFILTER noiseSensorGyroXBiNTF[NOISE_SENSOR_NTF_GYRO_FREQUENCY_N][NOISE_SENSOR_NTF_GYRO_HARMONIC_N];
BIQUADFILTER noiseSensorGyroYBiNTF[NOISE_SENSOR_NTF_GYRO_FREQUENCY_N][NOISE_SENSOR_NTF_GYRO_HARMONIC_N];
BIQUADFILTER noiseSensorGyroZBiNTF[NOISE_SENSOR_NTF_GYRO_FREQUENCY_N][NOISE_SENSOR_NTF_GYRO_HARMONIC_N];
//Low pass filters
		LOWPASSFILTER noiseSensorTempLPF;
		LOWPASSFILTER noiseSensorMagXLPF,
noiseSensorMagYLPF, noiseSensorMagZLPF;

void updateNoise(float dt) {
#if NOISE_SENSOR_LPF_ACC_ENABLED == 1 || NOISE_SENSOR_NTF_GYRO_ENABLED == 1
	fftX.fftDt = dt;
	fftY.fftDt = dt;
	fftZ.fftDt = dt;
	updateFFT(&fftX, attitudeData.axGRaw);
	updateFFT(&fftY, attitudeData.ayGRaw);
	updateFFT(&fftZ, attitudeData.azGRaw);
#endif
}

void filterAccNoise(float dt) {
#if NOISE_SENSOR_LPF_ACC_ENABLED == 1
	attitudeData.axG = lowPassFilterUpdate(&noiseSensorAccXLpF, attitudeData.axG, dt);
	attitudeData.ayG = lowPassFilterUpdate(&noiseSensorAccYLpF, attitudeData.ayG, dt);
	attitudeData.azG = lowPassFilterUpdate(&noiseSensorAccZLpF, attitudeData.azG, dt);
#endif
#if NOISE_SENSOR_NTF_ACC_ENABLED == 1
	for (uint8_t fIndx = 0; fIndx < NOISE_SENSOR_NTF_ACC_FREQUENCY_N; fIndx++) {
		uint8_t fIndx1 = fIndx + 1;
		float freqX = fftX.topFreq[fIndx] * fIndx1;
		float freqY = fftY.topFreq[fIndx] * fIndx1;
		float freqZ = fftZ.topFreq[fIndx] * fIndx1;
		for (int hIndx = 0; hIndx < NOISE_SENSOR_NTF_ACC_HARMONIC_N; hIndx++) {
			if (freqX >= NOISE_SENSOR_NTF_ACC_MIN_FREQUENCY && freqX <= NOISE_SENSOR_NTF_ACC_MAX_FREQUENCY) {
				biQuadFilterSetCenterFreq(&noiseSensorAccXBiNTF[fIndx][hIndx], freqX);
				attitudeData.axG = biQuadFilterUpdate(&noiseSensorAccXBiNTF[fIndx][hIndx], attitudeData.axG);
			}
			if (freqY >= NOISE_SENSOR_NTF_ACC_MIN_FREQUENCY && freqY <= NOISE_SENSOR_NTF_ACC_MAX_FREQUENCY) {
				biQuadFilterSetCenterFreq(&noiseSensorAccYBiNTF[fIndx][hIndx], freqY);
				attitudeData.ayG = biQuadFilterUpdate(&noiseSensorAccYBiNTF[fIndx][hIndx], attitudeData.ayG);
			}
			if (freqZ >= NOISE_SENSOR_NTF_ACC_MIN_FREQUENCY && freqZ <= NOISE_SENSOR_NTF_ACC_MAX_FREQUENCY) {
				biQuadFilterSetCenterFreq(&noiseSensorAccZBiNTF[fIndx][hIndx], freqZ);
				attitudeData.azG = biQuadFilterUpdate(&noiseSensorAccZBiNTF[fIndx][hIndx], attitudeData.azG);
			}
		}
	}
#endif
}

void filterGyroNoise(float dt) {
#if NOISE_SENSOR_LPF_GYRO_ENABLED == 1
	attitudeData.gxDS = lowPassFilterUpdate(&noiseSensorGyroXLpF, attitudeData.gxDS, dt);
	attitudeData.gyDS = lowPassFilterUpdate(&noiseSensorGyroYLpF, attitudeData.gyDS, dt);
	attitudeData.gzDS = lowPassFilterUpdate(&noiseSensorGyroZLpF, attitudeData.gzDS, dt);
#endif
#if NOISE_SENSOR_NTF_GYRO_ENABLED == 1
	for (uint8_t fIndx = 0; fIndx < NOISE_SENSOR_NTF_GYRO_FREQUENCY_N; fIndx++) {
		uint8_t fIndx1 = fIndx + 1;
		for (int hIndx = 0; hIndx < NOISE_SENSOR_NTF_GYRO_HARMONIC_N; hIndx++) {
			float freqX = fftX.topFreq[fIndx] * fIndx1;
			float freqY = fftY.topFreq[fIndx] * fIndx1;
			float freqZ = fftZ.topFreq[fIndx] * fIndx1;
			if (freqX >= NOISE_SENSOR_NTF_GYRO_MIN_FREQUENCY && freqX <= NOISE_SENSOR_NTF_GYRO_MAX_FREQUENCY) {
				biQuadFilterSetCenterFreq(&noiseSensorGyroXBiNTF[fIndx][hIndx], freqX);
				attitudeData.gxDS = biQuadFilterUpdate(&noiseSensorGyroXBiNTF[fIndx][hIndx], attitudeData.gxDS);
			}
			if (freqY >= NOISE_SENSOR_NTF_GYRO_MIN_FREQUENCY && freqY <= NOISE_SENSOR_NTF_GYRO_MAX_FREQUENCY) {
				biQuadFilterSetCenterFreq(&noiseSensorGyroYBiNTF[fIndx][hIndx], freqY);
				attitudeData.gyDS = biQuadFilterUpdate(&noiseSensorGyroYBiNTF[fIndx][hIndx], attitudeData.gyDS);
			}
			if (freqZ >= NOISE_SENSOR_NTF_GYRO_MIN_FREQUENCY && freqZ <= NOISE_SENSOR_NTF_GYRO_MAX_FREQUENCY) {
				biQuadFilterSetCenterFreq(&noiseSensorGyroZBiNTF[fIndx][hIndx], freqZ);
				attitudeData.gzDS = biQuadFilterUpdate(&noiseSensorGyroZBiNTF[fIndx][hIndx], attitudeData.gzDS);
			}
		} //Harmonic
	} //Frequency
#endif
}

void filterMagNoise(float dt) {
	attitudeData.mx = lowPassFilterUpdate(&noiseSensorMagXLPF, memsData.mx, dt);
	attitudeData.my = lowPassFilterUpdate(&noiseSensorMagYLPF, memsData.my, dt);
	attitudeData.mz = lowPassFilterUpdate(&noiseSensorMagZLPF, memsData.mz, dt);
}

void calculateTempCorrectionOffsets(float currentTemp) {
#if NOISE_SENSOR_TEMP_CORRECTION_ACC_ENABLED ==1 || NOISE_SENSOR_TEMP_CORRECTION_GYRO_ENABLED ==1
//temp_bias[0] = gyro_coeff_x[0] + gyro_coeff_x[1] * t +  gyro_coeff_x[2] * powf(t,2) + gyro_coeff_x[3] * powf(t,3);
	float tempP1 = (currentTemp - memsData.offsetTemp);
	float tempP2 = tempP1 * tempP1;
	float tempP3 = tempP2 * tempP1;
#if NOISE_SENSOR_TEMP_CORRECTION_ACC_ENABLED ==1
	memsData.accXTempOffset = memsData.accXTempCoeff[0] + memsData.accXTempCoeff[1] * tempP1 + memsData.accXTempCoeff[2] * tempP2 + memsData.accXTempCoeff[3] * tempP3;
	memsData.accYTempOffset = memsData.accYTempCoeff[0] + memsData.accYTempCoeff[1] * tempP1 + memsData.accYTempCoeff[2] * tempP2 + memsData.accYTempCoeff[3] * tempP3;
	memsData.accZTempOffset = memsData.accZTempCoeff[0] + memsData.accZTempCoeff[1] * tempP1 + memsData.accZTempCoeff[2] * tempP2 + memsData.accZTempCoeff[3] * tempP3;
#endif
#if NOISE_SENSOR_TEMP_CORRECTION_GYRO_ENABLED ==1
	memsData.gyroXTempOffset = memsData.gyroXTempCoeff[0] + memsData.gyroXTempCoeff[1] * tempP1 + memsData.gyroXTempCoeff[2] * tempP2 + memsData.gyroXTempCoeff[3] * tempP3;
	memsData.gyroYTempOffset = memsData.gyroYTempCoeff[0] + memsData.gyroYTempCoeff[1] * tempP1 + memsData.gyroYTempCoeff[2] * tempP2 + memsData.gyroYTempCoeff[3] * tempP3;
	memsData.gyroZTempOffset = memsData.gyroZTempCoeff[0] + memsData.gyroZTempCoeff[1] * tempP1 + memsData.gyroZTempCoeff[2] * tempP2 + memsData.gyroZTempCoeff[3] * tempP3;
#endif
#endif
}

void filterTempNoise(float dt) {
	attitudeData.temp = lowPassFilterUpdate(&noiseSensorTempLPF, memsData.tempC, dt);
	calculateTempCorrectionOffsets(attitudeData.temp);
}

uint8_t initNoiseSensor(float accSampleFrequency, float gyroSampleFrequency, float magSampleFrequency, float tempSampleFrequency) {
	initFFT(accSampleFrequency);
	initFFTContext(&fftX);
	initFFTContext(&fftY);
	initFFTContext(&fftZ);
	lowPassFilterInit(&noiseSensorMagXLPF, NOISE_SENSOR_LPF_MAG_FREQUENCY);
	lowPassFilterInit(&noiseSensorMagYLPF, NOISE_SENSOR_LPF_MAG_FREQUENCY);
	lowPassFilterInit(&noiseSensorMagZLPF, NOISE_SENSOR_LPF_MAG_FREQUENCY);
	lowPassFilterInit(&noiseSensorTempLPF, NOISE_SENSOR_LPF_TEMP_FREQUENCY);
#if NOISE_SENSOR_LPF_ACC_ENABLED == 1
	lowPassFilterInit(&noiseSensorAccXLpF, NOISE_SENSOR_LPF_ACC_FREQUENCY);
	lowPassFilterInit(&noiseSensorAccYLpF, NOISE_SENSOR_LPF_ACC_FREQUENCY);
	lowPassFilterInit(&noiseSensorAccZLpF, NOISE_SENSOR_LPF_ACC_FREQUENCY);
#endif
#if NOISE_SENSOR_LPF_GYRO_ENABLED == 1
	lowPassFilterInit(&noiseSensorGyroXLpF, NOISE_SENSOR_LPF_GYRO_FREQUENCY);
	lowPassFilterInit(&noiseSensorGyroYLpF, NOISE_SENSOR_LPF_GYRO_FREQUENCY);
	lowPassFilterInit(&noiseSensorGyroZLpF, NOISE_SENSOR_LPF_GYRO_FREQUENCY);
#endif

#if NOISE_SENSOR_NTF_ACC_ENABLED == 1
	for (int fIndx = 0; fIndx < NOISE_SENSOR_NTF_ACC_FREQUENCY_N; fIndx++) {
		for (int hIndx = 0; hIndx < NOISE_SENSOR_NTF_ACC_HARMONIC_N; hIndx++) {
			biQuadFilterInit(&noiseSensorAccXBiNTF[fIndx][hIndx], BIQUAD_NOTCH, NOISE_SENSOR_NTF_ACC_MIN_FREQUENCY, accSampleFrequency, NOISE_SENSOR_NTF_ACC_Q_ARRAY[fIndx][hIndx], NOISE_SENSOR_NTF_ACC_PEAK_GAIN);
			biQuadFilterInit(&noiseSensorAccYBiNTF[fIndx][hIndx], BIQUAD_NOTCH, NOISE_SENSOR_NTF_ACC_MIN_FREQUENCY, accSampleFrequency, NOISE_SENSOR_NTF_ACC_Q_ARRAY[fIndx][hIndx], NOISE_SENSOR_NTF_ACC_PEAK_GAIN);
			biQuadFilterInit(&noiseSensorAccZBiNTF[fIndx][hIndx], BIQUAD_NOTCH, NOISE_SENSOR_NTF_ACC_MIN_FREQUENCY, accSampleFrequency, NOISE_SENSOR_NTF_ACC_Q_ARRAY[fIndx][hIndx], NOISE_SENSOR_NTF_ACC_PEAK_GAIN);
		}
	}
#endif
#if NOISE_SENSOR_NTF_GYRO_ENABLED == 1
	for (int fIndx = 0; fIndx < NOISE_SENSOR_NTF_GYRO_FREQUENCY_N; fIndx++) {
		for (int hIndx = 0; hIndx < NOISE_SENSOR_NTF_GYRO_HARMONIC_N; hIndx++) {
			biQuadFilterInit(&noiseSensorGyroXBiNTF[fIndx][hIndx], BIQUAD_NOTCH, NOISE_SENSOR_NTF_GYRO_MIN_FREQUENCY, gyroSampleFrequency, NOISE_SENSOR_NTF_GYRO_Q_ARRAY[fIndx][hIndx], NOISE_SENSOR_NTF_GYRO_PEAK_GAIN);
			biQuadFilterInit(&noiseSensorGyroYBiNTF[fIndx][hIndx], BIQUAD_NOTCH, NOISE_SENSOR_NTF_GYRO_MIN_FREQUENCY, gyroSampleFrequency, NOISE_SENSOR_NTF_GYRO_Q_ARRAY[fIndx][hIndx], NOISE_SENSOR_NTF_GYRO_PEAK_GAIN);
			biQuadFilterInit(&noiseSensorGyroZBiNTF[fIndx][hIndx], BIQUAD_NOTCH, NOISE_SENSOR_NTF_GYRO_MIN_FREQUENCY, gyroSampleFrequency, NOISE_SENSOR_NTF_GYRO_Q_ARRAY[fIndx][hIndx], NOISE_SENSOR_NTF_GYRO_PEAK_GAIN);
		}
	}
#endif
	return 1;
}

void resetNoiseSensor() {
	lowPassFilterReset(&noiseSensorAccXLpF);
	lowPassFilterReset(&noiseSensorAccYLpF);
	lowPassFilterReset(&noiseSensorAccZLpF);
	lowPassFilterReset(&noiseSensorGyroXLpF);
	lowPassFilterReset(&noiseSensorGyroYLpF);
	lowPassFilterReset(&noiseSensorGyroZLpF);
	lowPassFilterReset(&noiseSensorMagXLPF);
	lowPassFilterReset(&noiseSensorMagYLPF);
	lowPassFilterReset(&noiseSensorMagZLPF);
	lowPassFilterReset(&noiseSensorTempLPF);
	for (int fIndx = 0; fIndx < NOISE_SENSOR_NTF_ACC_FREQUENCY_N; fIndx++) {
		for (int hIndx = 0; hIndx < NOISE_SENSOR_NTF_ACC_HARMONIC_N; hIndx++) {
			biQuadFilterReset(&noiseSensorAccXBiNTF[fIndx][hIndx]);
			biQuadFilterReset(&noiseSensorAccYBiNTF[fIndx][hIndx]);
			biQuadFilterReset(&noiseSensorAccZBiNTF[fIndx][hIndx]);
		}
	}
	for (int fIndx = 0; fIndx < NOISE_SENSOR_NTF_GYRO_FREQUENCY_N; fIndx++) {
		for (int hIndx = 0; hIndx < NOISE_SENSOR_NTF_GYRO_HARMONIC_N; hIndx++) {
			biQuadFilterReset(&noiseSensorGyroXBiNTF[fIndx][hIndx]);
			biQuadFilterReset(&noiseSensorGyroYBiNTF[fIndx][hIndx]);
			biQuadFilterReset(&noiseSensorGyroZBiNTF[fIndx][hIndx]);
		}
	}
}
