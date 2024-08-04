#ifndef _VIBRATIONSENSOR_H_
#define _VIBRATIONSENSOR_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "fft.h"
#include "memsCommon.h"


#define NOISE_SENSOR_LPF_ACC_ENABLED 1 //Anti Aliasing
#define NOISE_SENSOR_LPF_ACC_FREQUENCY MEMS_ACC_SAMPLE_FREQUENCY/2.0f //Cut off above nyngustic frequency
#define NOISE_SENSOR_NTF_ACC_ENABLED 1
#define NOISE_SENSOR_NTF_ACC_FREQUENCY_N 1 //Top Frequencies
#define NOISE_SENSOR_NTF_ACC_HARMONIC_N 3 //Harmonic levels

#define NOISE_SENSOR_NTF_ACC_MIN_FREQUENCY 100.0f
#define NOISE_SENSOR_NTF_ACC_MAX_FREQUENCY 800.0f
#define NOISE_SENSOR_NTF_ACC_PEAK_GAIN 1.0f
#define NOISE_SENSOR_NTF_ACC_Q0 0.925f
#define NOISE_SENSOR_NTF_ACC_Q1 0.925f
#define NOISE_SENSOR_NTF_ACC_Q2 0.925f

#define NOISE_SENSOR_LPF_GYRO_ENABLED 1 //Anti Aliasing
#define NOISE_SENSOR_LPF_GYRO_FREQUENCY MEMS_ACC_SAMPLE_FREQUENCY/2.0f //Cut off above nyngustic frrequency
#define NOISE_SENSOR_NTF_GYRO_ENABLED 1
#define NOISE_SENSOR_NTF_GYRO_FREQUENCY_N 1 //Top Frequencies
#define NOISE_SENSOR_NTF_GYRO_HARMONIC_N 3 //Harmonic levels

#define NOISE_SENSOR_NTF_GYRO_MIN_FREQUENCY 100.0f
#define NOISE_SENSOR_NTF_GYRO_MAX_FREQUENCY 800.0f
#define NOISE_SENSOR_NTF_GYRO_PEAK_GAIN 1.0f
#define NOISE_SENSOR_NTF_GYRO_Q0 0.925f
#define NOISE_SENSOR_NTF_GYRO_Q1 0.925f
#define NOISE_SENSOR_NTF_GYRO_Q2 0.925f

#define NOISE_SENSOR_LPF_MAG_FREQUENCY  100  //100
#define NOISE_SENSOR_LPF_TEMP_FREQUENCY 20  //10
#define NOISE_SENSOR_TEMP_CORRECTION_ACC_ENABLED MEMS_APPLY_ACC_TEMP_OFFSET_CORRECTION
#define NOISE_SENSOR_TEMP_CORRECTION_GYRO_ENABLED MEMS_APPLY_GYRO_TEMP_OFFSET_CORRECTION

extern FFTContext fftX;
extern FFTContext fftY;
extern FFTContext fftZ;

uint8_t initNoiseSensor(float accSampleFrequency, float gyroSampleFrequency, float magSampleFrequency, float tempSampleFrequency);
void resetNoiseSensor(void);
void updateNoise(float dt);
void filterTempNoise(float dt);
void filterAccNoise(float dt);
void filterGyroNoise(float dt);
void filterMagNoise(float dt);

#endif
