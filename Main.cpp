
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

uint16_t setPoint = 0;
uint16_t dcFront, dcRight, dcLeft, dcBack;
uint8_t dcFrontLow,dcFrontHigh , dcLeftLow, dcLeftHigh, dcRightLow, dcRightHigh, dcBackLow, dcBackHigh;
uint8_t slave = 0x40<<1; // 8 bit addr
uint16_t duty_cycle = 0;
uint8_t dc_low_byte = 0;
uint8_t dc_high_byte = 0;
uint16_t DC_MIN = 180;
uint16_t DC_MIN_Running = 200;
uint16_t DC_MAX_Running = 350;
uint16_t DC_MAX = 430;

struct motorThrottle_struct {
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
} motorThrottle;

struct remoteThrottle_struct {
   uint8_t Throttle;
   uint8_t Yaw;
   uint8_t Pitch;
   uint8_t Roll;
} remoteThrottle;

typedef enum { //enumeration is a numbered list of variable
//   GPS_Q,
   IMU_Q,
   controller_Q
} sharedHandleId_t;
#define quesize 100

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
   dcFront = DC_MIN_Running + motorThrottle.throttle + motorThrottle.pitchPIDv; //- motorThrottle.yawPIDv;

   dcBack = DC_MIN_Running + motorThrottle.throttle - motorThrottle.pitchPIDv; //- motorThrottle.yawPIDv;

   dcLeft = DC_MIN_Running + motorThrottle.throttle - motorThrottle.rollPIDv; //+ motorThrottle.yawPIDv;

   dcRight = DC_MIN_Running + motorThrottle.throttle + motorThrottle.rollPIDv; //+ motorThrottle.yawPIDv;

//   printf(" \nF: %i", dcFront);
//   printf(" B: %i", dcBack);
//   printf(" L: %i", dcLeft);
//   printf(" R: %i", dcRight);

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

   dcRightLow = dcRight;
   dcRightHigh = dcRight >> 8;
   dcLeftLow = dcLeft;
   dcLeftLow = dcLeft >> 8;
   dcBackLow = dcBack;
   dcBackHigh = dcBack >> 8;
   dcFrontLow = dcFront;
   dcFrontHigh = dcFront >> 8;


//   printf(" FA: %i", dcFront);
//   printf(" BA: %i", dcBack);
//   printf(" LA: %i", dcLeft);
//   printf(" RA: %i", dcRight);

   //FRONT
   i2c.writeReg(slave, 58, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 59, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 60, dcFrontLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 61, dcFrontHigh); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 46, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 47, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 48, dcFrontLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 49, dcFrontHigh); //high byte
   delay_ms(1);
   //BACK
   i2c.writeReg(slave, 14, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 15, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 16, dcBackLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 17, dcBackHigh); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 22, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 23, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 24, dcBackLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 25, dcBackHigh); //high byte
   //RIGHT
   i2c.writeReg(slave, 38, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 39, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 40, dcRightLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 41, dcRightHigh); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 30, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 31, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 32, dcRightLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 33, dcRightHigh); //high byte
   delay_ms(1);
   //LEFT
   i2c.writeReg(slave, 6, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 7, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 8, dcLeftLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 9, dcLeftHigh); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 66, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 67, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 68, dcLeftLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 69, dcLeftHigh); //high byte
   delay_ms(1);
}

class readController : public scheduler_task {
    public:
        readController(uint8_t priority) : scheduler_task("readController", 2048, priority) {
        QueueHandle_t contQ = xQueueCreate(1, quesize); //create queue
        addSharedObject(controller_Q, contQ); //add my sensor to the queue
    };
    bool run(void *p) {
        TickType_t xLastWakeTime;
        const TickType_t xFrequency = 200;
        xLastWakeTime = xTaskGetTickCount();
        char valueC[quesize] = {0};
        Uart2& u2= Uart2::getInstance();
        u2.init(9600); //baud rate
        u2.putChar('k');
        u2.gets(&valueC[0],quesize,100);
        xQueueSend(getSharedObject(controller_Q), &valueC, portMAX_DELAY); //sends the data to the queue to be processed
        vTaskDelayUntil( &xLastWakeTime, xFrequency/portTICK_PERIOD_MS );
        return true;
    }
};

 class processController : public scheduler_task
 {
     public:
         processController(uint8_t priority) : scheduler_task("processController", 4096, priority) { }
         bool run(void *p) {
             char valuebufferC[quesize] = {0};
             char *tokensC[4];
             float throttle = 0;
             float pitchC = 0;
             float rollC = 0;
             int k =0;
             QueueHandle_t contQ = getSharedObject(controller_Q); //create the q
             if (xQueueReceive(contQ, &valuebufferC, portMAX_DELAY)) {
                 char *ptrC;
                 ptrC = strtok(valuebufferC,":");
                 while (ptrC) {
                     tokensC[k++] = ptrC;
                     ptrC = strtok(NULL, ":");
                 }
                //printf("\n%s",valuebufferC);
                //stick ranges from 0-1024, so divide by 10 to get roughly 0-100%
                throttle = atoi(tokensC[1]);
                //float yawC = (str::toFloat(tokensC[2]))/10;
                pitchC = atoi(tokensC[2]);
                rollC = atoi(tokensC[3]);
                remoteThrottle.Throttle = throttle;
                remoteThrottle.Pitch = pitchC;
                remoteThrottle.Roll = rollC;
                printf("\n Throttle: %i", remoteThrottle.Lift);
                printf("Pitch: %i", remoteThrottle.Pitch);
                printf("Roll: %i", remoteThrottle.Roll);

                if((throttle< 65) | (throttle >45)) { //because sticks don't sit exactly at 500
                    throttle = 50;
                }
                if (throttle >= 85) {
                    throttle = 85;
                }
                if((pitchC< 65) | (pitchC >45)) { //because sticks don't sit exactly at 500
                    pitchC = 0; //zero percent pitch
                }

                if (pitchC > 65){
                    pitchC = 2;
                }

                if (pitchC < 45){
                    pitchC = -2;
                }
                if ((rollC< 65) | (rollC >45)) { //because sticks don't sit exactly at 500
                    rollC = 0; //zero percent roll
                }

                if(rollC > 65){
                    rollC = 2;
                }

                if(rollC < 45){
                    rollC = -2;
                }
             }
         return true;
         }
 };

//class readGPS : public scheduler_task {
//    public:
//        readGPS(uint8_t priority) : scheduler_task("fetchGPS", 2048, priority) {
//        QueueHandle_t q = xQueueCreate(1, quesize); //create queue
//        addSharedObject(GPS_Q, q); //add my sensor to the queue
//    };
//    bool run(void *p) {
//        TickType_t xLastWakeTime;
//        const TickType_t xFrequency = 200;
//        xLastWakeTime = xTaskGetTickCount();
//        char value[quesize] = {0};
//        Uart3& u3 = Uart3::getInstance();
//        u3.init(57600); //baud rate
//        u3.gets(&value[0],quesize,100);
//        puts(value);
//        xQueueSend(getSharedObject(GPS_Q), &value, portMAX_DELAY); //sends the data to the queue to be processed
//        vTaskDelayUntil( &xLastWakeTime, xFrequency/portTICK_PERIOD_MS );
//        return true;
//    }
//};
// class calculateGPS : public scheduler_task
// {
//     public:
//         calculateGPS(uint8_t priority) : scheduler_task("parseGPS", 2048, priority) { }
//         bool run(void *p) {
//             char valuebuffer[quesize] = {0};
//             QueueHandle_t q_name = getSharedObject(GPS_Q); //create the q
//             if (xQueueReceive(q_name, &valuebuffer, portMAX_DELAY)) {
//                printf("\n");
//                puts(valuebuffer);
//                printf("\n");
//                //TODO finish the parsing
//                }
//             return true;
//         }
// };
//void GPS_init(void) {
//    char init_response[quesize] = {0};
//    Uart3& u3 = Uart3::getInstance();
//    u3.init(9600); //baud rate
//    const char baud_57600[] = "$PMTK251,57600*2C";
//    const char output_RMCGGA[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\n";
//    const char update_1hz[]= "$PMTK220,1000*1F\n";
//    const char update_5hz[]=  "$PMTK220,200*2C";
//    const char antenna[] = "$PGCMD,33,1*6C\n";
//    const char no_antenna[] =  "$PGCMD,33,0*6D";
//    //request release and version number
//    //const char PMTK_Q_RELEASE[] = "PMTK605*31\n";
//
//    //Set the output for RMC and GGA
//    u3.putline(output_RMCGGA);
//    u3.gets(&init_response[0],quesize,1000);
//    puts(init_response);
//    puts("\n");
//
//    //Set the output rate 1Hz
//    init_response[quesize] = {0};
//    u3.putline(update_5hz);
//    u3.gets(&init_response[0],quesize,1000);
//    puts(init_response);
//    puts("\n");
//
//    //Set the antenna type
//    init_response[quesize] = {0};
//    u3.putline(no_antenna);
//    u3.gets(&init_response[0],quesize,1000);
//    puts(init_response);
//    puts("\n");
//
//    //Set the output for RMC and GGA
//    u3.putline(baud_57600);
//    u3.gets(&init_response[0],quesize,1000);
//    puts(init_response);
//    puts("\n");
//
//    //Respond with init finish
//    puts("GPS Init complete");
//    puts("\n");
//
//}

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
                //motorThrottle.yawPIDv = pidController(setPoint,yawV,&yawPID);
                motorThrottle.pitchPIDv = pidController(setPoint,pitchV,&pitchPID);
                motorThrottle.rollPIDv = pidController(setPoint,rollV,&rollPID);
                //printf("\n YAW: %i",motorThrottle.yawPIDv);
                //printf("    PITCH: %i",motorThrottle.pitchPIDv);
                //printf("    ROLL: %i",motorThrottle.rollPIDv);
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
        printf("\nLoop %i", i);
        if(i == 0) {
            duty_cycle = DC_MIN;
            dc_low_byte = duty_cycle;
            dc_high_byte = duty_cycle >> 8;
        }
        if(i == 1) {
            duty_cycle = DC_MAX;
            dc_low_byte = duty_cycle;
            dc_high_byte = duty_cycle >> 8;
        }
        if(i == 2) {
            duty_cycle = DC_MIN;
            dc_low_byte = duty_cycle;
            dc_high_byte = duty_cycle >> 8;
        }
        for (int n = 6; n <= 66;) {
            printf("\nLoop %i", n);
            i2c.writeReg(slave, n, 0x0); //low byte
            delay_ms(1);
            i2c.writeReg(slave, n+1, 0x0); //high byte
            delay_ms(1);
            i2c.writeReg(slave, n+2, dc_low_byte); //low byte
            delay_ms(1);
            i2c.writeReg(slave, n+3, dc_high_byte); //high byte
            delay_ms(1);
            n=n+4;
        }
    }
};


int main(void) {
    initializeMotors();
    pidInit(1,1,1,&yawPID);
    pidInit(1,1,1,&pitchPID);
    pidInit(1,1,1,&rollPID);
    printf("Started Loop");
    scheduler_add_task(new readIMU(PRIORITY_CRITICAL));
    scheduler_add_task(new calculateIMU(PRIORITY_HIGH));
    scheduler_add_task(new readController(PRIORITY_CRITICAL));
    scheduler_add_task(new processController(PRIORITY_MEDIUM));
    scheduler_start();
    return -1;
}
