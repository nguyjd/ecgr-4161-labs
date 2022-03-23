//**************************************************************
// LAB 6 - Use the ultrasonic sensor to find the wall, Drive 30cm to the wall
// Then rotate CW 90 degrees and drive for 100cm.
// Jonathon Nguyen, 03-21-2022
//**************************************************************
#include "SimpleRSLK.h"

// The parameters that the lab 6 is on.
#define DRIVEDISTANCE_CM 100.0  // How far to drive the robot.
#define SCAN_DEGREES 75.0       // FOV of the robot to find the wall
#define DEGREES_PER_SCAN 1.0    // How much to rotate per scan
#define ROTATE_DEGREES 90.0     // How much to turn CW
#define SAMPLES_PER_READING 15  // The amount of samples per Usonic reading
#define DISTANCE_FROM_WALL 30.0 // The distance from the wall to stop

// Stats for the Usonic
#define TRIGPIN 32              // Pin 5.1 on the board
#define ECHOPIN 33              // Pin 3.5 on the board
#define SENSOR_TIMEOUT 10000    // 10000 microseconds before the timeout.
#define MAX_DIST 400.0          // The max distance of the Usonic sensor.
#define SENSOR_OFFSET 1.0       // The distance from the front of the robot in cm

// Robot Dimensions/Stats
#define WHEEL_DIAMETER_CM 7.0
#define ROBOT_BASE_CM 14.0
#define PULSE_REVOLUTION 360.0
#define DRIVESPEED 13
#define PIVOTSPEED 7

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
  const uint16_t swayTolerance = 2;
  
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

  // sleep for 3 second to let the robot setting and measurement.
  delay(3000);
  
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

/*
 * FindClosestDistance
 * 
 * This function will rotate the robot and scan with the Usonic
 * Once the shortest distance is found. The robot will rotate to point to the closest object.
 */
void FindClosestDistance() {

  // Store the distance and degrees found.
  float shortestDistance = 9999.0;
  float shortestDegrees = 9999.0;

  // Scan the envirorment to find the shortest distance.
  float distance = 0.0;
  for (float i = 0; i <= SCAN_DEGREES; i += DEGREES_PER_SCAN) {

    // Read the sensor and rotate the robot.
    distance = ReadUltrasonic();
    RotateInPlace(DEGREES_PER_SCAN, true);

    if (distance < shortestDistance) {

      shortestDistance = distance;
      shortestDegrees = i;

      // Serial.print("New shortest distance found. Distance: ");
      // Serial.print(shortestDistance);
      // Serial.print(" @ Degrees: ");
      // Serial.println(shortestDegrees);

    }
    
  }

  // Rotate the robot to point to the wall.
  // Also perform a second scan.
  for (float i = SCAN_DEGREES; i >= 0; i -= DEGREES_PER_SCAN) {

      // Read the sensor and rotate the robot.
      distance = ReadUltrasonic();
      RotateInPlace(DEGREES_PER_SCAN, false);
  
      if (distance <= (shortestDistance) || i == shortestDegrees) {
  
        break;
  
      }
  }

  // Serial.println("");
  // Serial.print("Rotating back: ");
  // Serial.println(SCAN_DEGREES - shortestDegrees);

  delay(1000);

}

/*
 * DriveToWall
 * 
 * This function will drive to the wall.
 * The robot will stop a distance from the wall.
 */
void DriveToWall() {

  // Find the distance to drive to be 30 cm from the wall.
  float distanceToDrive = (ReadUltrasonic() - DISTANCE_FROM_WALL) - SENSOR_OFFSET;

  // Drive the distance to be 30 cm from the wall.
  DriveStraight(distanceToDrive);
  
  delay(5000);

}

void setup() { // put your setup code here, to run once:
  setupRSLK();

  // Setup the pins for the USonic
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

  // Serial.begin(9600);

  // Delay for 2 seconds so I can let go of the robot.
  delay(2000);
  
  // Run the algorithm of lab.
  FindClosestDistance();
  DriveToWall();
  RotateInPlace(ROTATE_DEGREES, false);
  DriveStraight(DRIVEDISTANCE_CM);

}

void loop() { // put your main code here, to run repeatedly: 
}
