#ifndef PID_H
#define PID_H
 
#include "stdint.h"
 
#define SCALING_FACTOR  128
 
typedef struct pidDATA{
  int16_t previousPWM;
  int32_t sumE;
  int16_t P;
  int16_t I;
  int16_t D;
  int16_t maxE;
  int32_t maxSumE;
};

struct pidDATA yawPID;
struct pidDATA pitchPID;
struct pidDATA rollPID;

#define MAX_INT         INT16_MAX
#define MAX_LONG        INT32_MAX
#define MAX_I_TERM      (MAX_LONG / 2)
#define FALSE           0
#define TRUE            1
 
void pid_Init(int16_t p, int16_t i, int16_t d, struct pidDATA *pid);
int16_t pid_Controller(int16_t setPoint, int16_t currentAngle, struct pidDATA *pid);
void pid_Reset_Integrator(struct pidDATA *pid);
 
#endif
