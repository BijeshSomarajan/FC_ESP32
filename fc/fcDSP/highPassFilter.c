#include "highPassFilter.h"

void highPassFilterInit(HIGHPASSFILTER* self, float cutoff) {
	self->cutOff = cutoff;
	self->rc = 1.0f / (self->cutOff * 2.0f * 3.14159f);
	self->pData = 0;
	self->output = 0;
}

void highPassFilterReset(HIGHPASSFILTER *self) {
	self->output = 0;
	self->pData = 0;
}

float highPassFilterUpdate(HIGHPASSFILTER* self, float data, float dt) {
	 float alpha = dt / (self->rc + dt);
	 self->output = alpha * (self->output + (data - self->pData));
	 return self->output;
}

