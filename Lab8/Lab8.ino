//******************************************************************
// LAB 8 - Drive down a hallway and find boxes on the right and left.
// Jonathon Nguyen, 04-18-2022
//******************************************************************
#include <Servo.h>
#include "SimpleRSLK.h"

// The parameters that the lab 8 is on.
#define DRIVE_STRAIGHT_CM 100     // How far to drive.
#define DISTANCE_PER_SCAN_CM 20   // How far to drive before scanning.
#define BOX_THRESHOLD 65.0       // The distance sensor see when its a box.
#define SAMPLES_PER_READING 15    // The amount of samples per Usonic reading

// Stats for the Servo Motor
#define SERVO_PIN 38              
Servo servoMotor;

// Stats for the Usonic
#define TRIGPIN 32                // Pin 5.1 on the board
#define ECHOPIN 33                // Pin 3.5 on the board
#define SENSOR_TIMEOUT 20000      // 20000 microseconds before the timeout.
#define MAX_DIST 400.0            // The max distance of the Usonic sensor.
#define SENSOR_OFFSET 1.0         // The distance from the front of the robot in cm

// Robot Dimensions/Stats
#define WHEEL_DIAMETER_CM 7.0
#define ROBOT_BASE_CM 14.0
#define PULSE_REVOLUTION 360.0
#define DRIVESPEED 13
#define PIVOTSPEED 7

// Arrays hold if a object is left or right
// first index, 0 - right, 1 - left
float distanceScans[2][DRIVE_STRAIGHT_CM/DISTANCE_PER_SCAN_CM + 1];
bool scans[2][DRIVE_STRAIGHT_CM/DISTANCE_PER_SCAN_CM + 1];

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
 * FindBoxesLeftandRight
 * 
 * This function find location of boxes.
 * The reading from the Usonic will be filtered using a threshold
 * This is part 1 of the lab.
 */
void FindBoxesLeftandRight() {

  int amountScan = DRIVE_STRAIGHT_CM/DISTANCE_PER_SCAN_CM;
  int rightDegrees = 0;
  int leftDegrees = 180;
  int straightDegrees = 90;
  int rotationDelay = 5;

  // Scanning and mapping.
  for (int scan = 0; scan <= amountScan; scan++) {

    // Move the servo and scan the right side.
    // The servo would not move with straight input, this is my solution.
    for (int angle = straightDegrees; angle >= rightDegrees; angle--) {

      servoMotor.write(angle);
      delay(rotationDelay);
      
    }
    float rightScan = ReadUltrasonic();
    distanceScans[0][scan] = rightScan;

    // Logic to determine if box is there.
    if (rightScan < BOX_THRESHOLD) {

      scans[0][scan] = true;
      
    } else {

      scans[0][scan] = false;
      
    }

    // Move the servo and scan the left side.
    for (int angle = rightDegrees; angle <= leftDegrees; angle++) {

      servoMotor.write(angle);
      delay(rotationDelay);
      
    }
    float leftScan = ReadUltrasonic();
    distanceScans[1][scan] = leftScan;

    // Logic to determine if box is there.
    if (leftScan < BOX_THRESHOLD) {

      scans[1][scan] = true;
      
    } else {

      scans[1][scan] = false;
      
    }

    for (int angle = leftDegrees; angle >= straightDegrees; angle--) {

      servoMotor.write(angle);
      delay(rotationDelay);
      
    }

    // Stop moving straight when at max distance.
    if (scan != amountScan) {

      DriveStraight(DISTANCE_PER_SCAN_CM);
      
    }
    
  }

  Serial.println("Scanning Results: Distance");
  for (int scan = 0; scan <= amountScan; scan++) {

    Serial.print("CM: ");
    Serial.print(scan*DISTANCE_PER_SCAN_CM);
    Serial.print(" | Left: ");
    Serial.print(distanceScans[1][scan]);
    Serial.print(" cm, Right: ");
    Serial.print(distanceScans[0][scan]);
    Serial.println(" cm");
    
  }

  Serial.println("Scanning Results: 1 for Box, 0 for no Box");
  for (int scan = 0; scan <= amountScan; scan++) {

    Serial.print("Boxes: ");
    Serial.print(scan*DISTANCE_PER_SCAN_CM);
    Serial.print(" | Left: ");
    Serial.print(scans[1][scan]);
    Serial.print(" , Right: ");
    Serial.println(scans[0][scan]);
    
  }
  
}

/*
 * StopAndLookAtBoxes
 * 
 * This function stop at each boxes and look at it.
 * The array is reverse ie right - 1, left - 0
 * This is part 2 of the lab.
 */
void StopAndLookAtBoxes() {  

  int amountScan = DRIVE_STRAIGHT_CM/DISTANCE_PER_SCAN_CM;
  int rightDegrees = 0;
  int leftDegrees = 180;
  int straightDegrees = 90;
  float distanceTraveled = 0;
  int rotationDelay = 5;

  // Turn around.
  RotateInPlace(180.0, true);

  for (int scan = amountScan; scan >= 0; scan--) {

    // I uses vars here just in case
    // that boxes are the same distance on both side.
    float leftDistance = 0;
    float rightDistance = 0;

    // Right side has a box.
    if (scans[1][scan] == true) {

      // look ahead and find mid.
      int lookAheadCounter = 1;
      while (true) {

        lookAheadCounter++;
        
        // Check to make sure the index is not out of bounds.
        if (scan - lookAheadCounter >= 0) {

          if (scans[1][scan - lookAheadCounter] == false) {

            // Backtrack one and break the loop.
            lookAheadCounter--;
            break;
            
          }
          
        } else {

          break;
          
        }
        
      }
      
      float middleIndex = (float)lookAheadCounter/2.0;

      float distanceFromStart = DRIVE_STRAIGHT_CM - scan*DISTANCE_PER_SCAN_CM;
      float distanceFromFirstIndex = middleIndex*DISTANCE_PER_SCAN_CM;

      rightDistance = (distanceFromStart + distanceFromFirstIndex) - distanceTraveled;
      
    }

    // Left side has a box.
    if (scans[0][scan] == true) {

      // look ahead and find mid.
      int lookAheadCounter = 1;
      while (true) {

        lookAheadCounter++;
          
        // Check to make sure the index is not out of bounds.
        if (scan - lookAheadCounter >= 0) {
  
          if (scans[1][scan - lookAheadCounter] == false) {
  
            // Backtrack one and break the loop.
            lookAheadCounter--;
            break;
              
          }
            
        } else {
  
          break;
            
        }
        
      }

      float middleIndex = (float)lookAheadCounter/2.0;

      float distanceFromStart = DRIVE_STRAIGHT_CM - scan*DISTANCE_PER_SCAN_CM;
      float distanceFromFirstIndex = middleIndex*DISTANCE_PER_SCAN_CM;

      rightDistance = (distanceFromStart + distanceFromFirstIndex) - distanceTraveled;
      
    }

    Serial.print("Scan ");
    Serial.println(scan);
    Serial.print("Left distance: ");
    Serial.print(leftDistance);
    Serial.print(" , Right distance: ");
    Serial.println(rightDistance);
    Serial.println("");

    // Both are greater than zero, we need to move twice probably.
    if (leftDistance > 0 && rightDistance > 0) {

      // The two distance are the same.
      if (abs(rightDistance - leftDistance) < 0.001) {

        DriveStraight(rightDistance);
        distanceTraveled += rightDistance;
        
        // Point left
        for (int angle = straightDegrees; angle <= leftDegrees; angle++) {

          servoMotor.write(angle);
          delay(rotationDelay);
      
        }

        delay(3000);
        
        // point right
        for (int angle = leftDegrees; angle >= rightDegrees; angle--) {

          servoMotor.write(angle);
          delay(rotationDelay);
      
        }

        delay(3000);

        // point straight
        for (int angle = rightDegrees; angle <= straightDegrees; angle++) {

          servoMotor.write(angle);
          delay(rotationDelay);
      
        }
        
      } 

      // The two distance are not the same.
      else {

        // Drive to left box first, then right.
        if (rightDistance > leftDistance) {
    
          DriveStraight(leftDistance);

          // Point left
          for (int angle = straightDegrees; angle <= leftDegrees; angle++) {

            servoMotor.write(angle);
            delay(rotationDelay);
      
          }

          delay(3000);
          DriveStraight(rightDistance - leftDistance);
          distanceTraveled += rightDistance;
          

          // Point right
          for (int angle = leftDegrees; angle >= rightDegrees; angle--) {

            servoMotor.write(angle);
            delay(rotationDelay);
      
          }

          delay(3000);

          // Point straight
          for (int angle = rightDegrees; angle <= straightDegrees; angle++) {

            servoMotor.write(angle);
            delay(rotationDelay);
      
          }
          
        } else {

          DriveStraight(rightDistance);

          // Point right
          for (int angle = straightDegrees; angle >= rightDegrees; angle--) {

            servoMotor.write(angle);
            delay(rotationDelay);
      
          }

          delay(3000);
          DriveStraight(leftDistance - rightDistance);
          distanceTraveled += leftDistance;
          

          // Point left
          for (int angle = rightDegrees; angle >= leftDegrees; angle++) {

            servoMotor.write(angle);
            delay(rotationDelay);
      
          }

          delay(3000);

          // Point straight
          for (int angle = leftDegrees; angle >= straightDegrees; angle--) {

            servoMotor.write(angle);
            delay(rotationDelay);
      
          }
          
        }
        
      }
      
    } 

    // One is zero and only need to travel to one box.
    else if(leftDistance > 0 || rightDistance > 0){

      // Go to the box on the right.
      if (rightDistance > 0) {
        
        DriveStraight(rightDistance);
        distanceTraveled += rightDistance;
          
        // Point at the right box.
        for (int angle = straightDegrees; angle >= rightDegrees; angle--) {

          servoMotor.write(angle);
          delay(rotationDelay);
      
        }

        delay(3000);

        // Point straight
        for (int angle = rightDegrees; angle <= straightDegrees; angle++) {

          servoMotor.write(angle);
          delay(rotationDelay);
      
        }
        
        
      }

      // The box is on the left.
      else {

        DriveStraight(leftDistance);
        distanceTraveled += leftDistance;
        
        // Point at the left box.
        for (int angle = straightDegrees; angle <= leftDegrees; angle++) {

          servoMotor.write(angle);
          delay(rotationDelay);
      
        }
        delay(3000);

        // Point straight
        for (int angle = leftDegrees; angle >= straightDegrees; angle--) {

          servoMotor.write(angle);
          delay(rotationDelay);
      
        }
        
      }
      
    }
    
  }

  // Drive the rest of the distance.
  if (distanceTraveled < DRIVE_STRAIGHT_CM) {

    DriveStraight(DRIVE_STRAIGHT_CM - distanceTraveled);
      
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
  FindBoxesLeftandRight();
  StopAndLookAtBoxes();

}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
