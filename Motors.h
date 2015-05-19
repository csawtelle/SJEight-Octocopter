struct motorThrottle_struct {
   uint16_t frontCCW;
   uint16_t frontCW;
   uint16_t leftCCW;
   uint16_t leftCW;
   uint16_t rightCCW;
   uint16_t rightCW;
   uint16_t backCCW;
   uint16_t backCW;
} motorThrottle; 
 
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

void motorPWM (uint16_t yawPIDv, uint16_t pitchPIDv, uint16_t rollPIDv) {
/*
      Front = Throttle + PitchPID - YawPID
      Back = Throttle - PitchPID - YawPID
      Left = Throttle + RollPID + YawPID
      Right = Throttle - RollPID + YawPID
*/

//The YAW probably wont work because the blades are all same rotation, needs to be fixed
   dcFront = (frontThrottle*4096)/100 + pitchPIDv - yawPIDv;
   dcFrontLow = dcFront;
   dcFrontHigh = dcFront >> 8;
   
   dcBack = (backThrottle*4096)/100 + pitchPIDv - yawPIDv;
   dcBackLow = dcBack;
   dcBackHigh = dcBack >> 8;
   
   dcLeft = (leftThrottle*4096)/100 + pitchPIDv - yawPIDv;
   dcLeftLow = dcLeft;
   dcLeftLow = dcLeft >> 8;
   
   dcRight = (rightThrottle*4096)/100 + pitchPIDv - yawPIDv;
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
