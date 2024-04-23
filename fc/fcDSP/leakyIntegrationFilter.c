#include "leakyIntegrationFlilter.h"

void leakyIntegrationFilterInit(LEAKYINTEGRATIONFILTER *self, float alhpa) {
	//A smaller value of alpha results in a slower decay and more persistence of past values
	//0.175f
	self->alhpa = alhpa;
}

void leakyIntegrationFilterReset(LEAKYINTEGRATIONFILTER *self, float value) {
	self->output = value;
}

float leakyIntegrationFilterUpdate(LEAKYINTEGRATIONFILTER *self, float input, float dt) {
	// Update the filtered value using leaky integration with time consideration
	float alpha_dt = self->alhpa * dt;
	self->output = alpha_dt * input + (1 - alpha_dt) * self->output;
	return self->output;
}
