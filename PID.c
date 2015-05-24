#include "pid.h"
#include "stdint.h"

void pidInit(int16_t p, int16_t i, int16_t d, struct pidDATA *pid) {
  pid->sumE = 0;
  pid->previousPWM = 0;
  pid->P = p;
  pid->I = i;
  pid->D = d;
  pid->maxE = MAX_INT / (pid->P + 1);
  pid->maxSumE = MAX_I_TERM / (pid->I + 1);
}

int16_t pidController(int16_t setPoint, int16_t currentPWM, struct pidDATA *pid) {
  int16_t error, pTerm, dTerm;
  int32_t iTerm, PWM, temp;

  error = (setPoint - currentPWM);

  if (error > pid->maxE) {
    pTerm = MAX_INT;
  }
  else if (error < -pid->maxE) {
    pTerm = -MAX_INT;
  }
  else {
    pTerm = pid-> P * error;
  }

  temp = pid->sumE + error;
  if(temp > pid->maxSumE) {
    iTerm = MAX_I_TERM;
    pid->sumE = pid->maxSumE;
  }
  else if(temp < -pid->maxSumE) {
    iTerm = -MAX_I_TERM;
    pid->sumE = -pid->maxSumE;
  }
  else {
    pid->sumE = temp;
    iTerm = pid->I * pid->sumE;
  }

  dTerm = pid->D * (pid->previousPWM - currentPWM);

  pid->previousPWM = currentPWM;

  PWM = (pTerm + iTerm + dTerm) / SCALING_FACTOR;
  if(PWM > MAX_INT) {
    PWM = MAX_INT;
  }
  else if(PWM < -MAX_INT) {
    PWM = -MAX_INT;
  }

  return((int16_t)PWM);
}

void pid_flush_I(struct pidDATA *pid)
{
  pid->sumE = 0;
}
