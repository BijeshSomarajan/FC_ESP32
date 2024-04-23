#include "pid.h"
/************************************************************************/
/* Initialization of the PID params                                     */
/************************************************************************/
void pidInit(PID *self, float p_kp, float p_ki, float p_kd, float dLPFk) {
	self->kp = p_kp;
	self->ki = p_ki;
	self->kd = p_kd;
	if (dLPFk != 0) {
		self->dLPFk = dLPFk;
		lowPassFilterInit(&self->dLPF, dLPFk);
	} else {
		self->dLPFk = 0;
	}
	self->pet = 0;
	self->i = 0;
}

/************************************************************************/
/* Set proportional gain                                                */
/************************************************************************/
void pidSetKP(PID *self, float p_kp) {
	self->kp = p_kp;
}

/************************************************************************/
/* Set integral gain                                                    */
/************************************************************************/
void pidSetKI(PID *self, float p_ki) {
	self->ki = p_ki;
}

/************************************************************************/
/* Set derivative gain                                                  */
/************************************************************************/
void pidSetKD(PID *self, float p_kd) {
	self->kd = p_kd;
}

/************************************************************************/
/* Set out put limits                                                   */
/************************************************************************/
void pidSetPIDOutputLimits(PID *self, float pidMin, float pidMax) {
	self->limitMax = pidMax;
	self->limitMin = pidMin;

	self->limitPMax = pidMax;
	self->limitPMin = pidMin;

	self->limitIMax = pidMax;
	self->limitIMin = pidMin;

	self->limitDMax = pidMax;
	self->limitDMin = pidMin;
}

/************************************************************************/
/* Set I out put limits                                                   */
/************************************************************************/
void pidSetPOutputLimits(PID *self, float p_Min, float p_Max) {
	self->limitPMax = p_Max;
	self->limitPMin = p_Min;
}

/************************************************************************/
/* Set I out put limits                                                   */
/************************************************************************/
void pidSetIOutputLimits(PID *self, float i_Min, float i_Max) {
	self->limitIMax = i_Max;
	self->limitIMin = i_Min;
}

/************************************************************************/
/* Set D out put limits                                                   */
/************************************************************************/
void pidSetDOutputLimits(PID *self, float d_Min, float d_Max) {
	self->limitDMax = d_Max;
	self->limitDMin = d_Min;
}

/************************************************************************/
/* Resets the PID ,integrated values                                   */
/************************************************************************/
void pidReset(PID *self) {
	self->pet = 0;
	self->i = 0;
	self->pid = 0;
	if (self->dLPFk != 0) {
		lowPassFilterReset(&self->dLPF);
	}
}

/************************************************************************/
/* Resets the PID ,I term                                               */
/************************************************************************/
void pidResetI(PID *self) {
	self->i = 0;
}

/**
 Resets the PID ,D term
 */
void pidResetD(PID *self) {
	self->pet = 0;
	if (self->dLPFk != 0) {
		lowPassFilterReset(&self->dLPF);
	}
}

/************************************************************************/
/*Calculates the PID                                                    */
/************************************************************************/
void pidUpdateWithGains(PID *self, float input, float sp, float dt, float kpGain, float kiGain, float kdGain) {
	float et = input - sp;
	//Proportional
	self->p = self->kp * kpGain * et;

	//Bound to limits
	if (self->p > self->limitPMax) {
		self->p = self->limitPMax;
	} else if (self->p < self->limitPMin) {
		self->p = self->limitPMin;
	}

	self->pid = self->p;

	//Integral
	if (self->ki != 0.0f) {
		self->i += et * dt;
		//Bound to limits
		if (self->i > self->limitIMax) {
			self->i = self->limitIMax;
		} else if (self->i < self->limitIMin) {
			self->i = self->limitIMin;
		}
		self->pid += self->ki * kiGain * self->i;
	}

	//Derivative
	if (self->kd != 0.0f) {
		float dTerm = 0;
		if (self->dLPFk != 0) {
			dTerm = self->kd * kdGain * lowPassFilterUpdate(&self->dLPF, ((et - self->pet) / dt), dt);
		} else {
			dTerm = self->kd * kdGain * ((et - self->pet) / dt);
		}
		//Bound to limits
		if (dTerm > self->limitDMax) {
			dTerm = self->limitDMax;
		} else if (dTerm < self->limitDMin) {
			dTerm = self->limitDMin;
		}
		self->pid += dTerm;
		self->pet = et;
	}

	//Limit Total Output to limits
	if (self->pid > self->limitMax) {
		self->pid = self->limitMax;
	} else if (self->pid < self->limitMin) {
		self->pid = self->limitMin;
	}

}

/************************************************************************/
/*Calculates the PID                                                    */
/************************************************************************/
void pidUpdate(PID *self, float input, float sp, float dt) {
	float et = input - sp;
	//Proportional
	self->p = self->kp * et;

	//Bound to limits
	if (self->p > self->limitPMax) {
		self->p = self->limitPMax;
	} else if (self->p < self->limitPMin) {
		self->p = self->limitPMin;
	}

	self->pid = self->p;

	//Integral
	if (self->ki != 0.0f) {
		self->i += et * dt;
		//Bound to limits
		if (self->i > self->limitIMax) {
			self->i = self->limitIMax;
		} else if (self->i < self->limitIMin) {
			self->i = self->limitIMin;
		}
		self->pid += self->ki * self->i;
	}

	//Derivative
	if (self->kd != 0.0f) {
		float dTerm = 0;
		if (self->dLPFk != 0) {
			dTerm = self->kd * lowPassFilterUpdate(&self->dLPF, ((et - self->pet) / dt), dt);
		} else {
			dTerm = self->kd * ((et - self->pet) / dt);
		}
		//Bound to limits
		if (dTerm > self->limitDMax) {
			dTerm = self->limitDMax;
		} else if (dTerm < self->limitDMin) {
			dTerm = self->limitDMin;
		}
		self->pid += dTerm;
		self->pet = et;
	}

	//Limit Total Output to limits
	if (self->pid > self->limitMax) {
		self->pid = self->limitMax;
	} else if (self->pid < self->limitMin) {
		self->pid = self->limitMin;
	}
}
