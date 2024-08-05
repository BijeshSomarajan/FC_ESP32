#ifndef _VIBRATIONSENSOR_H_
#define _VIBRATIONSENSOR_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "fft.h"
#include "memsCommon.h"

#define NOISE_SENSOR_LPF_ACC_ENABLED 1 //Anti Aliasing
#define NOISE_SENSOR_LPF_ACC_FREQUENCY 400.0f//MEMS_ACC_SAMPLE_FREQUENCY/2.0f //Should be less than nyngustic frequency
#define NOISE_SENSOR_NTF_ACC_ENABLED 1
#define NOISE_SENSOR_NTF_ACC_FREQUENCY_N 1 //Top Frequencies
#define NOISE_SENSOR_NTF_ACC_HARMONIC_N 3 //Harmonic levels

#define NOISE_SENSOR_NTF_ACC_MIN_FREQUENCY 100.0f
#define NOISE_SENSOR_NTF_ACC_MAX_FREQUENCY 800.0f
#define NOISE_SENSOR_NTF_ACC_PEAK_GAIN 2.0f
//#define NOISE_SENSOR_NTF_ACC_Q {{1.2f,1.0f,0.8f},{2.5f,2.0f,1.5f},{3.0f,2.5f,2.0f}}  //Works for 3/3
#define NOISE_SENSOR_NTF_ACC_Q {{0.75f,0.65f,0.55f},{1.0f,0.9f,0.8f},{1.2f,1.1f,1.0f}} //Works Only for 1/3

#define NOISE_SENSOR_LPF_GYRO_ENABLED 1 //Anti Aliasing
#define NOISE_SENSOR_LPF_GYRO_FREQUENCY 400.0f//MEMS_ACC_SAMPLE_FREQUENCY/2.0f //Should be less than nyngustic frequency
#define NOISE_SENSOR_NTF_GYRO_ENABLED 1
#define NOISE_SENSOR_NTF_GYRO_FREQUENCY_N 1 //Top Frequencies
#define NOISE_SENSOR_NTF_GYRO_HARMONIC_N 3//Harmonic levels

#define NOISE_SENSOR_NTF_GYRO_MIN_FREQUENCY 100.0f
#define NOISE_SENSOR_NTF_GYRO_MAX_FREQUENCY 800.0f
#define NOISE_SENSOR_NTF_GYRO_PEAK_GAIN 2.0f
//#define NOISE_SENSOR_NTF_GYRO_Q {{1.2f,1.0f,0.8f},{2.5f,2.0f,1.5f},{3.0f,2.5f,2.0f}}  //Works for 3/3
#define NOISE_SENSOR_NTF_GYRO_Q {{0.75f,0.65f,0.55f},{1.0f,0.9f,0.8f},{1.2f,1.1f,1.0f}} //Works Only for 1/3

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
