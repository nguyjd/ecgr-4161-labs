#include <Servo.h>

#define SECONDS 10
#define TOTAL_DEGREES 180
#define ROTATE_DEGREES 1

#define SERVO_PIN 38
Servo servoMotor;

void setup() {
  // put your setup code here, to run once:

  servoMotor.attach(SERVO_PIN);

}

void loop() {
  // put your main code here, to run repeatedly:

  servoMotor.write(0);

  delay(2000);

  for (int angle = 0; angle < TOTAL_DEGREES; angle += ROTATE_DEGREES) {

    servoMotor.write(angle);
    delay((1000 * SECONDS) / TOTAL_DEGREES);
    
  }

  delay(2000);
}
  
