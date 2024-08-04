#include "fft.h"
#include "mathUtil.h"
#include <math.h>
#include <stddef.h>
#include "lowPassFilter.h"
#include "commonUtil.h"

int fftLog2InputLength;
float fftSamplingFrequency;

FFTData fftTwiddleFactors[FFT_HALF_N];
float fftHannWindowValues[FFT_N];
float fftBinFrequencies[FFT_HALF_N + 1];
float fftBinWidth = 0, fftBinWidthHalf = 0;
//Calculate Log2 Length value
void calculateFFTLog2InputLength() {
	int logValue = 0;
	int n = FFT_N;
	while (n > 1) {
		n >>= 1; // Equivalent to n = n / 2
		logValue++;
	}
	fftLog2InputLength = logValue;
}

//Calculate HANN window values
void calculateFFTHannWindowValues() {
	for (int index = 0; index < FFT_N; index++) {
		fftHannWindowValues[index] = 0.5f * (1.0f - cosf(2.0f * PI_Val * index / (FFT_N - 1)));
	}
}

//Calculate Twiddle Factors
void calculateFFTTwiddleFactors() {
	for (int twiddleIndex = 0; twiddleIndex < FFT_HALF_N; twiddleIndex++) {
		float angle = -2.0f * PI_Val * twiddleIndex / FFT_N;
		fftTwiddleFactors[twiddleIndex].real = cosf(angle);
		fftTwiddleFactors[twiddleIndex].imag = sinf(angle);
	}
}

//Calculates the frequency bins
void calculateFFTOutputBinFrequencies() {
	for (int index = 0; index <= FFT_HALF_N; index++) {
		fftBinFrequencies[index] = (float) (index) * fftSamplingFrequency / FFT_N;
	}
}

// Initialize the FFT with the given sampling frequency
void initFFT(float sampleFrequency) {
	fftSamplingFrequency = sampleFrequency;
	fftBinWidth = fftSamplingFrequency / FFT_N;
	fftBinWidthHalf = fftBinWidth / 2.0f;
	calculateFFTTwiddleFactors();
	calculateFFTLog2InputLength();
	calculateFFTOutputBinFrequencies();
	calculateFFTHannWindowValues();
}

// Initialize the FFT context
void initFFTContext(FFTContext *ctx) {
	ctx->fftSampleCount = 0;
	//biQuadFilterInit(&ctx->lpfSample, BIQUAD_LOWPASS, fftSamplingFrequency / 2.0f, fftSamplingFrequency, FFT_LPF_NTF_Q, FFT_LPF_NTF_PEAK_GAIN);
	lowPassFilterInit(&ctx->lpfSample, fftSamplingFrequency / 2.0f);
}

// Function to perform the FFT
void performFFT(FFTContext *ctx) {
	// Bit-reverse the output array
	int reversedIndex = 0;
	for (int sampleIndex = 0; sampleIndex < FFT_N; sampleIndex++) {
		if (sampleIndex < reversedIndex) {
			FFTData temp = ctx->fftDataOut[sampleIndex];
			ctx->fftDataOut[sampleIndex] = ctx->fftDataOut[reversedIndex];
			ctx->fftDataOut[reversedIndex] = temp;
		}
		int bit = FFT_HALF_N;
		while (reversedIndex >= bit && bit >= 2) {
			reversedIndex -= bit;
			bit /= 2;
		}
		reversedIndex += bit;
	}
	// Perform the FFT
	for (int stage = 1; stage <= fftLog2InputLength; stage++) {
		int sectionSize = 1 << stage;
		int halfSectionSize = sectionSize / 2;
		int twiddleStep = FFT_N / sectionSize;
		for (int sectionStart = 0; sectionStart < FFT_N; sectionStart += sectionSize) {
			for (int subSectionIndex = 0; subSectionIndex < halfSectionSize; subSectionIndex++) {
				int twiddleIndex = subSectionIndex * twiddleStep;
				FFTData twiddleFactor = fftTwiddleFactors[twiddleIndex];
				FFTData temp = { twiddleFactor.real * ctx->fftDataOut[sectionStart + subSectionIndex + halfSectionSize].real - twiddleFactor.imag * ctx->fftDataOut[sectionStart + subSectionIndex + halfSectionSize].imag, twiddleFactor.real
						* ctx->fftDataOut[sectionStart + subSectionIndex + halfSectionSize].imag + twiddleFactor.imag * ctx->fftDataOut[sectionStart + subSectionIndex + halfSectionSize].real };
				FFTData butterflyTop = ctx->fftDataOut[sectionStart + subSectionIndex];
				ctx->fftDataOut[sectionStart + subSectionIndex] = (FFTData ) { butterflyTop.real + temp.real, butterflyTop.imag + temp.imag };
				ctx->fftDataOut[sectionStart + subSectionIndex + halfSectionSize] = (FFTData ) { butterflyTop.real - temp.real, butterflyTop.imag - temp.imag };
			}
		}
	}
}

// Function to accumulate a single sample and perform FFT if enough samples are accumulated
void updateFFT(FFTContext *ctx, float origSample) {
#if FFT_ENABLE_SAMPLE_LPF == 1
	float sample = lowPassFilterUpdate(&ctx->lpfSample, origSample, ctx->fftDt);
#else
	float sample = origSample;
#endif
#if FFT_ENABLE_WINDOWING == 1
	ctx->fftDataIn[ctx->fftSampleCount] = fftHannWindowValues[ctx->fftSampleCount] * sample;
#else
	ctx->fftDataIn[ctx->fftSampleCount] = sample;
#endif
	ctx->fftDataOut[ctx->fftSampleCount].real = ctx->fftDataIn[ctx->fftSampleCount];
	ctx->fftDataOut[ctx->fftSampleCount].imag = 0.0f;
	ctx->fftSampleCount++;
	if (ctx->fftSampleCount >= FFT_N) {
		performFFT(ctx);
		// Reset sample count for next batch
		ctx->fftSampleCount = 0;
		//Reset the magnitiudes
		for (int indx = 0; indx < 3; indx++) {
			ctx->topMagn[indx] = 0;
			ctx->topFreq[indx] = 0;
		}
		// Calculate magnitudes and corresponding frequencies of FFT bins
		for (int index = 1; index < FFT_HALF_N; index++) {
			ctx->fftMagnitudes[index] = fastSqrtf(ctx->fftDataOut[index].real * ctx->fftDataOut[index].real + ctx->fftDataOut[index].imag * ctx->fftDataOut[index].imag);
			//ctx->fftMagnitudes[index] = applyDeadBandFloat(0, ctx->fftMagnitudes[index], 0.02f);
			//Determine Top Frequencies
			if (ctx->topMagn[0] < ctx->fftMagnitudes[index]) {
				ctx->topMagn[2] = ctx->topMagn[1];
				ctx->topMagn[1] = ctx->topMagn[0];
				ctx->topMagn[0] = ctx->fftMagnitudes[index];
				ctx->topFreq[2] = ctx->topFreq[1];
				ctx->topFreq[1] = ctx->topFreq[0];
				ctx->topFreq[0] = fftBinFrequencies[index - 1] ;//+ fftBinWidthHalf;
			} else if (ctx->topMagn[1] < ctx->fftMagnitudes[index]) {
				ctx->topMagn[2] = ctx->topMagn[1];
				ctx->topMagn[1] = ctx->fftMagnitudes[index];
				ctx->topFreq[2] = ctx->topFreq[1];
				ctx->topFreq[1] = fftBinFrequencies[index - 1] ;//+ fftBinWidthHalf;
			} else if (ctx->topMagn[2] < ctx->fftMagnitudes[index]) {
				ctx->topMagn[2] = ctx->fftMagnitudes[index];
				ctx->topFreq[2] = fftBinFrequencies[index - 1] ;//+ fftBinWidthHalf;
				;
			}
		}
		bubbleSortDecF(ctx->topFreq, 3);
	}
}
