//**************************************************************
// LAB 5 - Drive the robot straight along the radius of a circle
// Then drive the circumference and return back to the center of the circle.
// Jonathon Nguyen, 02-28-2022
//**************************************************************
#include "SimpleRSLK.h"

// The parameters that the lab 5 is on.
#define DRIVEDISTANCE_CM 75
#define CIRCLERADIUS_CM 75
#define ROTATE_DEGREES 90

// Robot Dimensions/Stats
#define WHEEL_DIAMETER_CM 7.0
#define ROBOT_BASE_CM 14.0
#define PULSE_REVOLUTION 360
#define DRIVESPEED 15

/*
 * RotateInPlaceCCW
 * Params: float amountDegrees - the amount of degrees that the robot will rotate.
 * 
 * This function will calcuate the pulses to rotate the robot by the given degrees.
 * Then it will rotate the robot CCW while keeping the encoder equal to each other.
 */
void RotateInPlaceCCW(float amountDegrees) {

  // the difference that the right and left encoder count can be apart.
  const uint16_t swayTolerance = 1;

  // calculate the amount of encoder pulse require to rotate in place by amountDegrees
  float aroundCircle = (amountDegrees/360.0);
  uint16_t encoder = aroundCircle*(ROBOT_BASE_CM/WHEEL_DIAMETER_CM)*PULSE_REVOLUTION;

  // Set the speed of the motors and set the encodercount vars to zero.
  // the drivespeed is half to make the rotating the robot more consistant.
  uint16_t leftMotorSpeed = DRIVESPEED/2;
  uint16_t rightMotorSpeed = DRIVESPEED/2;
  uint16_t rightEncoderCount = 0;
  uint16_t leftEncoderCount = 0;

  // Set up the motors and reset the encoders
  resetLeftEncoderCnt(); 
  resetRightEncoderCnt();
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD); 
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD); 
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);          
  setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);

  // loop until both motors reach their calculated encoder count.
  while( (leftEncoderCount < encoder) || (rightEncoderCount < encoder) ) {
                         
      leftEncoderCount = getEncoderLeftCnt(); 
      rightEncoderCount = getEncoderRightCnt();
      
      if((leftEncoderCount + swayTolerance) < rightEncoderCount) {

        setMotorSpeed(LEFT_MOTOR, leftMotorSpeed++);
        
      } 
      else if ((rightEncoderCount + swayTolerance) < leftEncoderCount) {

        setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed++);
        
      } 
      else {

        rightMotorSpeed = DRIVESPEED/2;
        leftMotorSpeed = DRIVESPEED/2;

        setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);
        setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);
        
      }
      
      // Stop motors when they reach the distance.
      if (leftEncoderCount >= encoder) disableMotor(LEFT_MOTOR);
      if (rightEncoderCount >= encoder) disableMotor(RIGHT_MOTOR);
   }

   // sleep for 5 second to let the robot setting and measurement.
   delay(5000);
   
}

/*
 * DriveStraight
 * Params: float distanceCM - how far the robot will travel
 * 
 * This function will calcuate the amount of pulses to make the robot go forward.
 * The motor will be managed by checking the encoder.
 */
void DriveStraight(float distanceCM) {
  
  // the difference that the right and left encoder count can be apart.
  const uint16_t swayTolerance = 1;
  
  // calculate the amount of encoder pulse require to move staright by distanceCM
  uint16_t encoderAmount = (distanceCM/(WHEEL_DIAMETER_CM * PI))*PULSE_REVOLUTION;
  

  // Set the speed of the motors and set the encodercount vars to zero.
  uint16_t rightMotorSpeed = DRIVESPEED;
  uint16_t leftMotorSpeed = DRIVESPEED;
  uint16_t leftEncoderCount = 0;
  uint16_t rightEncoderCount = 0;

  // Set up the motors and reset the encoders
  resetLeftEncoderCnt(); 
  resetRightEncoderCnt();
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD); 
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);          
  setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);

  // loop until both motors reach their calculated encoder count.
  while( (leftEncoderCount < encoderAmount) || (rightEncoderCount < encoderAmount) ) {
                         
      leftEncoderCount = getEncoderLeftCnt(); 
      rightEncoderCount = getEncoderRightCnt();
      
      // if right motor is too fast, speed up the left motor 
      if((leftEncoderCount + swayTolerance) < rightEncoderCount) {

        setMotorSpeed(LEFT_MOTOR, leftMotorSpeed++);
        
      }
      else if ((rightEncoderCount + swayTolerance) < leftEncoderCount) {

        setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed++);
        
      }
      else {

        rightMotorSpeed = DRIVESPEED;
        leftMotorSpeed = DRIVESPEED;

        setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);
        setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);
        
      }
      
      // Stop motors when they reach the distance.
      if (leftEncoderCount >= encoderAmount) disableMotor(LEFT_MOTOR);
      if (rightEncoderCount >= encoderAmount) disableMotor(RIGHT_MOTOR);
      
  }

  // sleep for 5 second to let the robot setting and measurement.
  delay(5000);
  
}



/*
 * DriveInACircleCCW
 * Params: float diameterCM - The diameter of the circle the robot is driving.
 * 
 * This function will make the robot drive the perimeter of a circle. 
 * The motor will be managed by checking the encoder.
 */
void DriveInACircleCCW(float diameterCM) {

  // the difference that the right and left encoder count can be apart.
  const uint16_t swayTolerance = 3;

  // Determine the distance from the wheel to the center of the circle.
  float leftDistance = (diameterCM / 2.0) - (ROBOT_BASE_CM / 2.0);
  float rightDistance = (diameterCM / 2.0) + (ROBOT_BASE_CM / 2.0);

  // Find the amount of encoder pulse needed for each wheel to drive CCW in a circle.
  float leftEncoderAmount = ((leftDistance*2)/WHEEL_DIAMETER_CM) * PULSE_REVOLUTION;
  float rightEncoderAmount = ((rightDistance*2)/WHEEL_DIAMETER_CM) * PULSE_REVOLUTION;
  uint16_t leftCount = 0;
  uint16_t rightCount = 0;  

  // Create the wheel vars with the correct ratio of speed.
  float motorSpeedRatio = leftEncoderAmount/rightEncoderAmount;
  uint16_t rightMotorSpeed = DRIVESPEED;
  uint16_t leftMotorSpeed = motorSpeedRatio * rightMotorSpeed;
  
  // Set up the motors and reset the encoders
  resetLeftEncoderCnt(); 
  resetRightEncoderCnt();
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD); 
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);          
  setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);

  // loop until both motors reach their calculated encoder count.
  while( (leftCount < leftEncoderAmount) || (rightCount < rightEncoderAmount) ) {          
      leftCount = getEncoderLeftCnt(); 
      rightCount = getEncoderRightCnt();
      
      // if right motor is too fast, speed up the left motor 
      if((leftCount + swayTolerance) < (motorSpeedRatio * rightCount)) {

        setMotorSpeed(LEFT_MOTOR, leftMotorSpeed++);
        
      }
    // If the left mot0r is too fast, speed up the right motor.
      else if (((motorSpeedRatio * rightCount) + swayTolerance) < leftCount) {

        setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed++);
        
      }
    
      // Reset the speed when both motor are caught up.
      else {

        rightMotorSpeed = DRIVESPEED;
        leftMotorSpeed = motorSpeedRatio * rightMotorSpeed;

        setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);
        setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);
        
      }
      
      // Stop motors when they reach the distance.
      if (leftCount >= leftEncoderAmount) disableMotor(LEFT_MOTOR);
      if (rightCount >= rightEncoderAmount) disableMotor(RIGHT_MOTOR);
   }

  // sleep for 5 second to let the robot setting and measurement.
  delay(5000);
  
}

void setup() { // put your setup code here, to run once:
  setupRSLK();

  // Delay for 2 seconds so I can let go of the robot.
  delay(2000);
  
  // Run the algorithm of lab.
  DriveStraight(DRIVEDISTANCE_CM);
  RotateInPlaceCCW(ROTATE_DEGREES);
  DriveInACircleCCW(2 * CIRCLERADIUS_CM);
  RotateInPlaceCCW(ROTATE_DEGREES);
  DriveStraight(DRIVEDISTANCE_CM);
  
}

void loop() { // put your main code here, to run repeatedly: 
}
