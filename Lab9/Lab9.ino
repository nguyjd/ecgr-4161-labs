//******************************************************************
// LAB 9
// Jonathon Nguyen, 05-02-2022
//******************************************************************
#include <Servo.h>
#include "SimpleRSLK.h"

// The parameters that the lab 9 is on.
#define SAMPLES_PER_READING 5     // The amount of samples per Usonic reading
#define SAMPLE_LINE 100           // The amount of sample for line sensor
#define DEGREES_PER_SCAN 1.0      // How much to rotate per scan
#define SCAN_DEGREES 180.0        // FOV of the robot to find the wall
#define ROTATE_DEGREES 90.0       // How much to turn CW

// Stats for the Servo Motor
#define SERVO_PIN 35              // Pin 6.7 on the board           
Servo servoMotor;

// Stats for the Usonic
#define TRIGPIN 32                // Pin 5.1 on the board
#define ECHOPIN 33                // Pin 3.5 on the board
#define SENSOR_TIMEOUT 30000      // 30000 microseconds before the timeout.
#define MAX_DIST 400.0            // The max distance of the Usonic sensor.

// Robot Dimensions/Stats
#define WHEEL_DIAMETER_CM 7.0
#define ROBOT_BASE_CM 14.0
#define PULSE_REVOLUTION 360.0
#define DRIVESPEED 10
#define PIVOTSPEED 10

uint16_t lineSensor[LS_NUM_SENSORS];
uint16_t lineSensorCal[LS_NUM_SENSORS];
uint16_t lineSensorMax[LS_NUM_SENSORS];
uint16_t lineSensorMin[LS_NUM_SENSORS];

/*
 * RotateInPlace
 * Params: float amountDegrees - the amount of degrees that the robot will rotate.
 * Params: bool CCW - True if the robot need to rotate CCW.
 * 
 * This function will calcuate the pulses to rotate the robot by the given degrees.
 * Then it will rotate the robot CCW while keeping the encoder equal to each other.
 */
void RotateInPlace(float amountDegrees, bool CCW) {

  // the difference that the right and left encoder count can be apart.
  const uint16_t swayTolerance = 1;

  // calculate the amount of encoder pulse require to rotate in place by amountDegrees
  float aroundCircle = (amountDegrees/360.0);
  uint16_t encoder = aroundCircle*(ROBOT_BASE_CM/WHEEL_DIAMETER_CM)*PULSE_REVOLUTION;

  // Set the speed of the motors and set the encodercount vars to zero.
  uint16_t leftMotorSpeed = PIVOTSPEED;
  uint16_t rightMotorSpeed = PIVOTSPEED;
  uint16_t rightEncoderCount = 0;
  uint16_t leftEncoderCount = 0;

  // Set up the motors and reset the encoders
  resetLeftEncoderCnt(); 
  resetRightEncoderCnt();

  if (CCW) {

    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD); 
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD); 

  } else {

    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD); 
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD); 

  }
  
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

        rightMotorSpeed = PIVOTSPEED;
        leftMotorSpeed = PIVOTSPEED;

        setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);
        setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);
        
      }
      
      // Stop motors when they reach the distance.
      if (leftEncoderCount >= encoder) disableMotor(LEFT_MOTOR);
      if (rightEncoderCount >= encoder) disableMotor(RIGHT_MOTOR);
   }
   
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

  // The lowest that the speed can be.
  const uint16_t speedLowerLimit = 5;
  
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

        if (rightMotorSpeed - 1 < speedLowerLimit) {rightMotorSpeed--;}
        leftMotorSpeed++;
        
        
      }
      else if ((rightEncoderCount + swayTolerance) < leftEncoderCount) {

        if (leftMotorSpeed - 1 < speedLowerLimit) {leftMotorSpeed--;}
        rightMotorSpeed++;
        
      }
      else {

        rightMotorSpeed = DRIVESPEED;
        leftMotorSpeed = DRIVESPEED;
        
      }

      setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);
      setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);
      
      // Stop motors when they reach the distance.
      if (leftEncoderCount >= encoderAmount) disableMotor(LEFT_MOTOR);
      if (rightEncoderCount >= encoderAmount) disableMotor(RIGHT_MOTOR);
      
  }
  
}

/*
 * ReadUltrasonic
 * Returns: float The distance that the sensor reads
 * 
 * This function will sample the Usonic and store the result.
 * The results will be sorted and the mediean will be returned.
 */
float ReadUltrasonic() {

  float samples[SAMPLES_PER_READING];
  float pulseLength;

  // Sample the sensor.
  for (int i = 0; i < SAMPLES_PER_READING; i++) {

    // Trigger the sensor.
    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN, LOW);
    delayMicroseconds(10);

    // Parse the input by the Usonic.
    pulseLength = pulseIn(ECHOPIN, HIGH, SENSOR_TIMEOUT);

    // Save the result.
    if (pulseLength/58 == 0 || pulseLength/58 > MAX_DIST) {

      // Set to max distance so the short path function can work.
      samples[i] = MAX_DIST;
      
    } else {

      samples[i] = (pulseLength / 58);
      
    }

  }
  
  // Bubble Sorting
  // Loop through the entire array until there is no more swapping.
  bool inversion = true;
  while (inversion) { 
    
    inversion = false;
    for (int i = 0; i < SAMPLES_PER_READING - 1; i++)
    { 

      // Swap the values if the lesser index is less.
      if (samples[i] > samples[i + 1])
      { 

        float temp = samples[i];
        samples[i] = samples[i + 1];
        samples[i + 1] = temp;
        inversion = true;

      } 
    } 
  }

  // Return the median of the samples.
  return samples[SAMPLES_PER_READING / 2];
  
}

void LocateCenterOfRoom() {

  /* --Scan for the closest wall-- */
  // Store the distance and degrees found.
  float shortestDistance = 9999.0;
  float shortestDegrees = 9999.0;

  // Scan the envirorment to find the shortest distance.
  float distance = 0.0;
  
  for (float robotAngle = 0.0; robotAngle < 360.0; robotAngle += ROTATE_DEGREES) {

    distance = 0.0;
    for (float angle = 0; angle < SCAN_DEGREES; angle += DEGREES_PER_SCAN) {

    // Read the sensor and rotate the robot.
    distance = ReadUltrasonic();
    servoMotor.write(angle);

      if (distance < shortestDistance) {
  
        shortestDistance = distance;
        shortestDegrees = angle + robotAngle;
  
      }
      
    }

    servoMotor.write(0);
    delay(1000);
    RotateInPlace(ROTATE_DEGREES, true);
    
  }

  /*-- Point to the wall -- */
  // Check the degrees to see how to rotate.
  if (90 > shortestDegrees) {

    // In the first quadrant. Rotate the offset.
    RotateInPlace(90.0 - shortestDegrees, false);
      
  }
  else {

    RotateInPlace(shortestDegrees - 90.0, true);
      
  }

  servoMotor.write(90);
  delay(1000);

  /* --Find the midpoint-- */
  float x1 = ReadUltrasonic();
  RotateInPlace(180.0, true);
  float x2 = ReadUltrasonic();
  float midpoint = (x1 + x2) / 2;

  /* --Drive to the midpoint -- */
  delay(500);
  DriveStraight(x2 - midpoint);

  /* --Find the midpoint-- */
  // Turn 90 and scan
  delay(500);
  RotateInPlace(90.0, true);
  x1 = ReadUltrasonic();

  // Turn 180 and scan
  delay(500);
  RotateInPlace(180.0, true);
  x2 = ReadUltrasonic();
  midpoint = (x1 + x2) / 2;

  /* --Drive to the midpoint -- */
  if (x1 > x2) {

    RotateInPlace(180.0, true);
    delay(500);
    DriveStraight(x1 - midpoint);
    
  } 
  else {

    delay(500);
    DriveStraight(x2 - midpoint);
     
  }
  
}

void CalibrateLineSensors() {

  // Drive a little bit to calibrate the sensors.
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(BOTH_MOTORS, DRIVESPEED);

  // Sample the sensors
  for(int i = 0; i < SAMPLE_LINE; i++)
  {
    readLineSensor(lineSensor);
    setSensorMinMax(lineSensor, lineSensorMax, lineSensorMin);
  }

  // Turn off the motors.
  disableMotor(BOTH_MOTORS);
}

void FollowLine() {

  uint16_t normalSpeed = 15;
  uint16_t slowSpeed = 10;
  uint8_t lineColor = DARK_LINE;
  bool foundT = false;

  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(BOTH_MOTORS, normalSpeed);
  
  while (!foundT) {
    
    readLineSensor(lineSensor);
    readCalLineSensor(lineSensor,lineSensorCal,lineSensorMin,lineSensorMax,lineColor);
    uint32_t linePos = getLinePosition(lineSensorCal,lineColor);

    // Check the position of the line.
    // Pivot the robot to make the line in the middle.
    if(linePos > 0 && linePos < 3000) {
      
      setMotorSpeed(BOTH_MOTORS, slowSpeed);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      
    } 
    else if(linePos > 3500) 
    {
      setMotorSpeed(BOTH_MOTORS, slowSpeed);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    } 
    else 
    {
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    }
  
    foundT = true;
    // Check if all the sensor are filled.
    for (int i = 0; i < LS_NUM_SENSORS; i++)
    {
      
      if (lineSensorCal[i] < 800)
      {
  
        foundT = false;
        break;
        
      }
      
    }
    
  }

  disableMotor(BOTH_MOTORS);
  delay(2000);
  
}

void RotateToLine() {

  uint8_t lineColor = DARK_LINE;
  while (true) {

    readLineSensor(lineSensor);
    readCalLineSensor(lineSensor,lineSensorCal,lineSensorMin,lineSensorMax,lineColor);
    uint32_t linePos = getLinePosition(lineSensorCal, lineColor);
    
    if (lineSensorCal[3] == 1000 || lineSensorCal[4] == 1000) { break; }
    RotateInPlace(1.0, true);
    
  }
  
}

void MazeRunner() {

  const float turnLeftThresh = 60.0;
  const float turnRightThresh = 10.0;

  int rightDegrees = 0;
  int leftDegrees = 180;
  int straightDegrees = 90;
  int rotationDelay = 5;
  float distance = 0.0;

  while (true) {

    // Look straight for short distance
    for (int angle = leftDegrees; angle >= straightDegrees; angle--) {

      servoMotor.write(angle);
      delay(rotationDelay);
      
    }
    float straightdistance = ReadUltrasonic() - turnRightThresh;

    // look left for long distance
    for (int angle = straightDegrees; angle <= leftDegrees; angle++) {

      servoMotor.write(angle);
      delay(rotationDelay);
      
    }

    // Drive striaght
    bool turnLeft = false;
    for (float distanceTraveled = 0.0; distanceTraveled < straightdistance; 
         distanceTraveled += 1.0) {

      DriveStraight(1.0);

      distance = ReadUltrasonic();
  
      // There is a hole to turn left.
      if (distance > turnLeftThresh)
      {

        turnLeft = true;
        DriveStraight(ROBOT_BASE_CM + 10.0);
        RotateInPlace(90.0, true);
        DriveStraight(ROBOT_BASE_CM + 10.0);
        break;
        
      }
      
    }

    if (!turnLeft) {

      RotateInPlace(90.0, false);
      
    }
    
  }
  
}

void setup() {

  // Setup the pins for the servo motor.
  servoMotor.attach(SERVO_PIN);
  
  // put your setup code here, to run once:
  setupRSLK();

  Serial.begin(9600);

  // Setup the pins for the USonic
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

  // Delay for 2 seconds so I can let go of the robot.
  delay(2000);
  
  // Run the algorithm of lab.
  CalibrateLineSensors();
  delay(500);
  LocateCenterOfRoom();
  delay(500);
  RotateToLine();
  delay(500);
  FollowLine();
  delay(500);
  MazeRunner();

}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
