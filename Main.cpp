//Globals for now
    I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
    i2c.init(100);
    uint8_t slave = 0x40<<1; // 8 bit addr


typedef enum {
   yaw,
   pitch,
   roll
} axis_t

typedef enum { //enumeration is a numbered list of variables
   IMU_Q,
   controller_Q
} sharedHandleId_t;
#define quesize 100

void initializeMotors (void) {
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
            duty_cycle = (4*4096)/100;
            dc_low_byte = duty_cycle;
            dc_high_byte = duty_cycle >> 8;
        }
        if(i == 1) {
            duty_cycle = (10.5*4096)/100;
            dc_low_byte = duty_cycle;
            dc_high_byte = duty_cycle >> 8;
        }
        if(i == 2) {
            duty_cycle = (4*4096)/100;
            dc_low_byte = duty_cycle;
            dc_high_byte = duty_cycle >> 8;
        }
        for (int n = 6; n < 66;) {
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
            //send the off signal
        for (int n = 6; n < 66; n++) {
            i2c.writeReg(slave, n, 0); //off
            delay_ms(1);
        }
    }
}

void motorPWM (uint16_t yawPID, uint16_t pitchPID, uint16_t rollPID) {
/*

      Front = Throttle + PitchPID - YawPID
      Back = Throttle - PitchPID - YawPID
      Left = Throttle + RollPID + YawPID
      Right = Throttle - RollPID + YawPID

*/

//The YAW probably wont work because the blades are all same rotation, needs to be fixed
   dcFront = (frontThrottle*4096)/100 + pitchPID - yawPID;
   dcFrontLow = dcFront;
   dcFrontHigh = dcFront >> 8;
   
   dcBack = (backThrottle*4096)/100 + pitchPID - yawPID;
   dcBackLow = dcBack;
   dcBackHigh = dcBack >> 8;
   
   dcLeft = (leftThrottle*4096)/100 + pitchPID - yawPID;
   dcLeftLow = dcLeft;
   dcLeftLow = dcLeft >> 8;
   
   dcRight = (rightThrottle*4096)/100 + pitchPID - yawPID;
   dcRightLow = dcRight;
   dcRightHigh = dcRight >> 8;

   //Front
   i2c.writeReg(slave, 6, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 7, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 8, dcFrontLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 9, dcFrontHigh); //high byte
   delay_ms(1);
   //back
   i2c.writeReg(slave, 38, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 39, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 40, dcBackLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 41, dcBackHigh); //high byte
   delay_ms(1);
   //left
   i2c.writeReg(slave, 22, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 23, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 24, dcLeftLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 25, dcLeftHigh); //high byte
   delay_ms(1);
   //right
   i2c.writeReg(slave, 54, 0x0); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 55, 0x0); //high byte
   delay_ms(1);
   i2c.writeReg(slave, 56, dcRightLow); //low byte
   delay_ms(1);
   i2c.writeReg(slave, 57, dcRightHigh); //high byte
   delay_ms(1);
}
int main(void) {
   
   
   
    //Add tasks depending on which board you are loading the code too
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
     * Try the rx / tx tasks together to see how they queue data to each other.
     */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
