
#include "SimpleRSLK.h"
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

void setup()
{

  Serial.begin(9600);
  
  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
  clearMinMax(sensorMinVal,sensorMaxVal);
  
}
void floorCalibration() 
{
  /* Place Robot On Floor (no line) */
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  
  delay(1000);
  simpleCalibrate();
  
  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  
  delay(1000);
  enableMotor(BOTH_MOTORS);
}
void simpleCalibrate() 
{
  
  /* Set both motors direction forward */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS,20);
  for(int i = 0; i < 100; i++)
  {
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
  }
  
  /* Disable both motors */
  disableMotor(BOTH_MOTORS);
}
bool isCalibrationComplete = false;
bool foundT = false;

void loop() {

  uint16_t normalSpeed = 10;
  uint16_t fastSpeed = 20;
  /* Valid values are either:
   *  DARK_LINE  if your floor is lighter than your line
   *  LIGHT_LINE if your floor is darker than your line
   */
  uint8_t lineColor = DARK_LINE;
  
  /* Run this setup only once */
  if(isCalibrationComplete == false) {
    floorCalibration();
    isCalibrationComplete = true;
  }
  
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);
  uint32_t linePos = getLinePosition(sensorCalVal,lineColor);

  Serial.println("");
  Serial.println(linePos);
  
  if(linePos > 0 && linePos < 3000) {
  setMotorSpeed(LEFT_MOTOR,normalSpeed);
  setMotorSpeed(RIGHT_MOTOR,fastSpeed);
  } else if(linePos > 3500) {
  setMotorSpeed(LEFT_MOTOR,fastSpeed);
  setMotorSpeed(RIGHT_MOTOR,normalSpeed);
  } else {
  setMotorSpeed(LEFT_MOTOR,normalSpeed);
  setMotorSpeed(RIGHT_MOTOR,normalSpeed);
}

  foundT = true;
  for (int i = 0; i < LS_NUM_SENSORS; i++)
  {

    if (sensorCalVal[i] < 100)
    {

      foundT = false;
      break;
      
    }
    
  }

  while (foundT) {disableMotor(BOTH_MOTORS);}
  
}
