#ifndef FC_FCDSP_INCLUDE_FFT_H_
#define FC_FCDSP_INCLUDE_FFT_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "lowPassFilter.h"
#include "biQuadFilter.h"

#define FFT_N 128 // Must be a power of 2 // 128 Flies!
#define FFT_TOP_FREQ_N 3 // Must be a power of 2
#define FFT_HALF_N FFT_N/2 // Must be a power of 2

#define FFT_ENABLE_SAMPLE_LPF 1
#define FFT_ENABLE_WINDOWING 1

// Define the FFT data structure
typedef struct _FFTData FFTData;
struct _FFTData {
	float real;
	float imag;
};

// FFT context structure to hold data arrays and relevant variables
typedef struct _FFTContext FFTContext;
struct _FFTContext {
	float fftDataIn[FFT_N];
	FFTData fftDataOut[FFT_N];
	float fftMagnitudes[FFT_HALF_N + 1];
	float topFreq[FFT_TOP_FREQ_N];
	float topMagn[FFT_TOP_FREQ_N];
	int fftSampleCount;
	float fftDt;
	LOWPASSFILTER lpfSample;
};

void initFFT(float sampleFrequency);
void initFFTContext(FFTContext *ctx);
void updateFFT(FFTContext *ctx, float origSample);

#endif
