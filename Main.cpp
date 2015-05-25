
#include "lpc_rit.h"
#include "tasks.hpp"
#include "stdio.h"
#include "LPC17xx.h"
#include <FreeRTOS.h>
#include <task.h>
#include <utilities.h>
#include <uart0_min.h>
#include "i2c2.hpp"
#include <stdint.h>
#include <stdio.h>
#include "io.hpp"
#include "semphr.h"
#include "eint.h"
#include "PID.h"
#include "src/PID.c"
#include "string.h"
#include "str.hpp"
#include "uart2.hpp"
#include "uart3.hpp"
#include "iostream"
#include <stdlib.h>     /* strtol */

uint16_t setPoint = 0;
uint16_t dcFront, dcRight, dcLeft, dcBack;
uint8_t dcFrontLow,dcFrontHigh , dcLeftLow, dcLeftHigh, dcRightLow, dcRightHigh, dcBackLow, dcBackHigh;
uint8_t slave = 0x40<<1; // 8 bit addr
uint16_t duty_cycle = 0;
uint16_t DC_MIN_Running = 250;
uint16_t DC_MAX_Running = 430;

uint16_t DC_MIN = 180;
uint16_t DC_MAX = 430;

uint16_t RC_MIN = 450;
uint16_t RC_MAX = 550;

uint8_t RC_DIV = 1;

struct MotorThrottle_struct {
   uint16_t frontCCW;
   uint16_t frontCW;
   uint16_t leftCCW;
   uint16_t leftCW;
   uint16_t rightCCW;
   uint16_t rightCW;
   uint16_t backCCW;
   uint16_t backCW;
   uint16_t throttle;
   int16_t yawPIDv;
   int16_t pitchPIDv;
   int16_t rollPIDv;
} MT;

struct RemoteControl_struct {
   int16_t Throttle;
   int16_t Yaw;
   int16_t Pitch;
   int16_t Roll;
} RC;

typedef enum { //enumeration is a numbered list of variable
//   GPS_Q,
   IMU_Q,
   controller_Q
} sharedHandleId_t;
#define quesize 100

void setPWM(uint8_t channel, uint16_t dutyCycle) {
    I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
    i2c.init(100);
    uint8_t lowbyte = dutyCycle;
    uint8_t highbyte = dutyCycle>>8;
    channel = 6+(4*channel);
    i2c.writeReg(slave, channel, 0x0); //low byte
    delay_ms(1);
    i2c.writeReg(slave, channel+1, 0x0); //high byte
    delay_ms(1);
    i2c.writeReg(slave, channel+2, lowbyte); //low byte
    delay_ms(1);
    i2c.writeReg(slave, channel+3, highbyte); //high byte
};

void motorPWM () {
    I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
    i2c.init(100);
/*
      Front = Throttle + PitchPID - YawPID
      Back = Throttle - PitchPID - YawPID
      Left = Throttle - RollPID + YawPID
      Right = Throttle + RollPID + YawPID
*/
//The YAW probably wont work because the blades are all same rotation, needs to be fixed
   dcFront = DC_MIN_Running + RC.Throttle - RC.Pitch - MT.pitchPIDv; //- MT.yawPIDv;
   dcBack = DC_MIN_Running + RC.Throttle + RC.Pitch + MT.pitchPIDv; //- MT.yawPIDv;
   dcLeft = DC_MIN_Running + RC.Throttle - RC.Roll - MT.rollPIDv; //+ MT.yawPIDv;
   dcRight = DC_MIN_Running + RC.Throttle + RC.Roll + MT.rollPIDv; //+ MT.yawPIDv;

//    printf(" \nF: %i", dcFront);
//    printf(" B: %i", dcBack);
//    printf(" L: %i", dcLeft);
//    printf(" R: %i", dcRight);

   //FLOOR VALUES =============================
   if (dcFront < DC_MIN_Running) { dcFront = DC_MIN_Running; }
   if (dcBack < DC_MIN_Running) { dcBack = DC_MIN_Running; }
   if (dcLeft < DC_MIN_Running) { dcLeft = DC_MIN_Running; }
   if (dcRight < DC_MIN_Running) { dcRight = DC_MIN_Running; }
   //CIELING VALUES ==============================
   if (dcFront > DC_MAX_Running) { dcFront = DC_MAX_Running; }
   if (dcBack > DC_MAX_Running) { dcBack = DC_MAX_Running; }
   if (dcLeft > DC_MAX_Running) { dcLeft = DC_MAX_Running; }
   if (dcRight > DC_MAX_Running ) { dcRight = DC_MAX_Running; }

//   printf(" FA: %i", dcFront);
//   printf(" BA: %i", dcBack);
//   printf(" LA: %i", dcLeft);
//   printf(" RA: %i", dcRight);

   setPWM( 13, dcFront);
   setPWM( 10, dcFront);
   setPWM( 2, dcBack);
   setPWM( 4, dcBack);
   setPWM( 8, dcRight);
   setPWM( 6, dcRight);
   setPWM( 0, dcLeft);
   setPWM( 15, dcLeft);
}

class readController : public scheduler_task {
    public:
        readController(uint8_t priority) : scheduler_task("readController", 1024, priority) {
        QueueHandle_t cont = xQueueCreate(1, quesize); //create queue
        addSharedObject(controller_Q, cont); //add my sensor to the queue
    };
    bool run(void *p) {
        TickType_t xLastWakeTime;
        const TickType_t xFrequency = 200;
        xLastWakeTime = xTaskGetTickCount();
        char valueC[quesize] = {0};
        Uart2& u2= Uart2::getInstance();
        u2.init(38400); //baud rate
        u2.putChar('k'); //required to request data from the remote
        u2.gets(&valueC[0],quesize,100);
        xQueueSend(getSharedObject(controller_Q), &valueC, portMAX_DELAY); //sends the data to the queue to be processed
        vTaskDelayUntil( &xLastWakeTime, xFrequency/portTICK_PERIOD_MS );
        return true;
    }
};

 class processController : public scheduler_task
 {
     public:
         processController(uint8_t priority) : scheduler_task("processController", 1024, priority) { }
         bool run(void *p) {
             char valuebufferC[quesize] = {0};
             char *tokensC[4];
             int k = 0;
             QueueHandle_t contQ = getSharedObject(controller_Q); //create the q
             if (xQueueReceive(contQ, &valuebufferC, portMAX_DELAY)) {
                 char *ptrk;
                 ptrk = strtok(valuebufferC,":");
                 while (ptrk) {
                     tokensC[k++] = ptrk;
                     ptrk = strtok(NULL, ":");
                 }
                 float temp1 = str::toFloat(tokensC[0]);
                 float temp2 = str::toFloat(tokensC[1]);
                 float temp3 = str::toFloat(tokensC[2]);
                 float temp4 = str::toFloat(tokensC[3]);
                 RC.Throttle = temp1;
                 RC.Yaw = temp2;
                 RC.Pitch = temp3;
                 RC.Roll = temp4;

                //stick ranges from 0-1024, so divide by 10 to get roughly 0-100%
                if((RC.Throttle < RC_MAX) && (RC.Throttle > RC_MIN)) { //because sticks don't sit exactly at 500
                    RC.Throttle = 0;
                }
                if (RC.Throttle >= RC_MAX) {
                    RC.Throttle = (RC.Throttle>>RC_DIV);
                }
                if (RC.Throttle <= RC_MIN) {
                    RC.Throttle = 0-(RC.Throttle>>RC_DIV);
                }

                if((RC.Pitch < RC_MAX) && (RC.Pitch > RC_MIN)) { //because sticks don't sit exactly at 500
                    RC.Pitch = 0; //zero percent pitch
                }
                if (RC.Pitch >= RC_MAX) {
                    RC.Pitch = RC.Pitch>>RC_DIV;
                }
                if (RC.Pitch <= RC_MIN) {
                    RC.Pitch = 0-(RC.Pitch>>RC_DIV);
                }

                if ((RC.Roll < RC_MAX) && (RC.Roll > RC_MIN)) { //because sticks don't sit exactly at 500
                    RC.Roll = 0; //zero percent roll
                }
                if(RC.Roll >= RC_MAX) {
                    RC.Roll = (RC.Roll>>RC_DIV);
                }
                if(RC.Roll <= RC_MIN) {
                    RC.Roll = 0-(RC.Roll>>RC_DIV);
                }

                if ((RC.Yaw < RC_MAX) && (RC.Yaw > RC_MIN)) { //because sticks don't sit exactly at 500
                    RC.Yaw = 0; //zero percent yaw
                }
                if(RC.Yaw >= RC_MAX) {
                    RC.Yaw = RC.Yaw>>RC_DIV;
                }
                if(RC.Yaw <= RC_MIN) {
                    RC.Yaw = 0-(RC.Yaw>>RC_DIV);
                }
             }
         return true;
         }
 };

 class readIMU : public scheduler_task {
    public:
        readIMU(uint8_t priority) : scheduler_task("read_AHREF", 2048, priority) {
        QueueHandle_t q = xQueueCreate(1, quesize); //create queue
        addSharedObject(IMU_Q, q); //add my sensor to the queue
    };
    bool run(void *p) {

        TickType_t xLastWakeTime;
        const TickType_t xFrequency = 20;
        xLastWakeTime = xTaskGetTickCount();
        char value[quesize] = {0};
        Uart3& u3 = Uart3::getInstance();
        u3.init(115200); //baud rate
        u3.putline("#s<12>");
        u3.gets(&value[0],quesize,100);
        xQueueSend(getSharedObject(IMU_Q), &value, portMAX_DELAY); //sends the data to the queue to be processed
        vTaskDelayUntil( &xLastWakeTime, xFrequency/portTICK_PERIOD_MS );
        return true;
    }
};

class calculateIMU : public scheduler_task
 {
     public:
         calculateIMU(uint8_t priority) : scheduler_task("calculate_heading", 2048, priority) { }
         bool run(void *p) {
             char valuebuffer[quesize] = {0};
             char *tokens[4];
             float yawV=0;
             float pitchV=0;
             float rollV=0;
             int j =0;
             QueueHandle_t q_name = getSharedObject(IMU_Q); //create the q
             if (xQueueReceive(q_name, &valuebuffer, portMAX_DELAY)) {
                char *ptr;
                ptr = strtok(valuebuffer,",");
                while (ptr) {
                    tokens[j++] = ptr;
                    ptr = strtok(NULL, ",");
                }
                yawV = str::toFloat(tokens[1]);
                pitchV = str::toFloat(tokens[2]);
                rollV = str::toFloat(tokens[3]);
                printf("\n YAW: %f", yawV);
                printf("  PITCH: %f", pitchV);
                printf("  ROLL: %f", rollV);
                MT.yawPIDv = pidController(setPoint,yawV,&yawPID);
                MT.pitchPIDv = pidController(setPoint,pitchV,&pitchPID);
                MT.rollPIDv = pidController(setPoint,rollV,&rollPID);
//                printf("\n YAW: %i", MT.yawPIDv);
//                printf("  PITCH: %i", MT.pitchPIDv);
//                printf("  ROLL: %i", MT.rollPIDv);
                motorPWM();
             }
             return true;
         }
 };
void initializeMotors (void) {
    I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
    i2c.init(100);
      uint8_t mode1_reg = 0x0;
      uint8_t prescale_reg = 0xFE;
      uint8_t prescale_value = 0x80;
      uint8_t sleep = 0x10;
      uint8_t initialize = 0xA1;
    //initialize
    i2c.writeReg(slave, mode1_reg, sleep); //sleep
    delay_ms(1);
    i2c.writeReg(slave, prescale_reg, prescale_value); //set clock
    delay_ms(5);
    i2c.writeReg(slave, mode1_reg, initialize); //wake up, get going
    delay_ms(1);
    //send the maximum signal
    for(int i = 0; i<3;i++) {
        if(i == 0) {
            duty_cycle = DC_MIN;
        }
        if(i == 1) {
            duty_cycle = DC_MAX;
        }
        if(i == 2) {
            duty_cycle = DC_MIN;
        }
        for (int n=0; n<=15 ;n++ ) {
            setPWM(n,duty_cycle);
        }
    }
};


int main(void) {
    printf("\nStart");
    initializeMotors();
    pidInit(1,1,1,&yawPID);
    pidInit(1,1,1,&pitchPID);
    pidInit(1,1,1,&rollPID);
    printf("\nSchedule");
    scheduler_add_task(new readIMU(PRIORITY_CRITICAL));
    scheduler_add_task(new calculateIMU(PRIORITY_HIGH));
    scheduler_add_task(new readController(PRIORITY_CRITICAL));
    scheduler_add_task(new processController(PRIORITY_MEDIUM));
    printf("\nBegin Code");
    scheduler_start();
    return -1;
}
