#include "SimpleRSLK.h"
#define WHEELSPEED 40
#define WHEELSPEED_70 30
#define WHEELSPEED_80 20
#define PULSES_100CM 1637

void straight_1_meter_adjust (void) { 
   // Define speed and encoder count variables
   uint16_t l_motor_speed = WHEELSPEED;
   uint16_t r_motor_speed = WHEELSPEED;
   uint16_t straight = PULSES_100CM;
   uint16_t l_totalCount = 0;
   uint16_t r_totalCount = 0; 
   
   // Set up the motors and encoders
   resetLeftEncoderCnt();  resetRightEncoderCnt();   // Set encoder pulse count back to 0 
   setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward 
   enableMotor(BOTH_MOTORS);                         // "Turn on" the motor 
   setMotorSpeed(LEFT_MOTOR, l_motor_speed);         // Set motor speeds - variable, 
   setMotorSpeed(RIGHT_MOTOR, r_motor_speed);        //   may change (adjust) later
   // Drive both motors until both have received the correct number of pulses to travel
   while( (l_totalCount<straight) || (r_totalCount<straight) ) {                     
      l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
      
      // if right motor is too fast, speed up the left motor 
      if((l_totalCount+1) < r_totalCount) {
        setMotorSpeed(LEFT_MOTOR, ++l_motor_speed);
        setMotorSpeed(RIGHT_MOTOR, --r_motor_speed);
      }else if((r_totalCount+1) < l_totalCount) {
        setMotorSpeed(RIGHT_MOTOR, ++r_motor_speed);
        setMotorSpeed(LEFT_MOTOR, --l_motor_speed);
      }
      else
      {
        if (straight * .70 < l_totalCount)
        {

          l_motor_speed = WHEELSPEED_70;
          r_motor_speed = WHEELSPEED_70;
          
        } else if (straight * .80 < l_totalCount)
        {

          l_motor_speed = WHEELSPEED_80;
          r_motor_speed = WHEELSPEED_80;
          
        }
        else
        {

          l_motor_speed = WHEELSPEED;
          r_motor_speed = WHEELSPEED;
          
        }
        
        
        setMotorSpeed(RIGHT_MOTOR, r_motor_speed);
        setMotorSpeed(LEFT_MOTOR, l_motor_speed);
        
      }
      // Stop motors if they reach 1 meter
      if (l_totalCount >= straight) disableMotor(LEFT_MOTOR);
      if (r_totalCount >= straight) disableMotor(RIGHT_MOTOR);
   }
}

void setup() {
  // put your setup code here, to run once:
  setupRSLK();
  
  delay(2000);
  straight_1_meter_adjust();

}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
