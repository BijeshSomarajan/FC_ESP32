#ifndef CORE_DSP_FILTER_LEAKYINTEGRATIONFLILTER_H_
#define CORE_DSP_FILTER_LEAKYINTEGRATIONFLILTER_H_

typedef struct _LEAKYINTEGRATIONFILTER LEAKYINTEGRATIONFILTER;

struct _LEAKYINTEGRATIONFILTER {
	float alhpa;
	float output;
};

void leakyIntegrationFilterInit(LEAKYINTEGRATIONFILTER *self, float alhpa);
void leakyIntegrationFilterReset(LEAKYINTEGRATIONFILTER *self,float value);
float leakyIntegrationFilterUpdate(LEAKYINTEGRATIONFILTER *self, float input, float dt);

#endif /* CORE_DSP_FILTER_LEAKYINTEGRATIONFLILTER_H_ */
