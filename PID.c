#include "pid.h"
#include "stdint.h"
 
void pidInit(int16_t p, int16_t i, int16_t d, struct pidDATA *pid) {
  pid->sumE = 0;
  pid->previousPWM = 0;
  pid->pGain = p;
  pid->iGain = i;
  pid->dGain = d;
  pid->maxE = MAX_INT / (pid->p + 1);
  pid->maxsumE = MAX_I_TERM / (pid->i + 1);
}
 
int16_t pidController(int16_t set, int16_t currentPWM, struct pidDATA *pid) {
  int16_t error, pTerm, dTerm;
  int32_t iTerm, PWM, temp;
 
  error = setPoint - currentPWM;
 
  if (error > pid_st->maxE) {
    pTerm = MAX_INT;
  }
  else if (error < -pid_st->maxE) {
    pTerm = -MAX_INT;
  }
  else {
    pTerm = pid_st->p * error;
  }
 
  temp = pid_st->sumError + error;
  if(temp > pid_st->maxSumE) {
    iTerm = MAX_I_TERM;
    pid_st->sumE = pid_st->maxSumE;
  }
  else if(temp < -pid_st->maxSumError) {
    iTerm = -MAX_I_TERM;
    pid_st->sumE = -pid_st->maxSumE;
  }
  else {
    pid_st->sumE = temp;
    iTerm = pid_st->i * pid_st->sumE;
  }
 
  dTerm = pid_st->d * (pid_st->previousPWM - currentPWM);
 
  pid_st->previousPWM = currentPWM;
 
  PWM = (pTerm + iTerm + dTerm) / SCALING_FACTOR;
  if(PWM > MAX_INT) {
    PWM = MAX_INT;
  }
  else if(PWM < -MAX_INT) {
    PWM = -MAX_INT;
  }
 
  return((int16_t)PWM);
}
 
void pid_flush_I(pidData_t *pid)
{
  pid_st->sumE = 0;
}
