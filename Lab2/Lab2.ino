//**************************************************************
// LAB 2 - Blink the leds in eight-state pattern
// Jonathon Nguyen, 01-31-2022
//**************************************************************
#define RED 75    // Define RED of the tri-color LED as pin 75
#define GREEN 76  // Define GREEN of the tri-color LED as pin 76
#define BLUE 77   // Define BLUE of the tri-color LED as pin 77

void setup() {        // put your setup code here, to run once:

  // Set the pin mode to the leds
  pinMode(RED, OUTPUT);       // Red LED 
  pinMode(GREEN, OUTPUT);     // Green LED
  pinMode(BLUE, OUTPUT);      // Blue LED
  
}

void loop() {    // put your main code here, to run repeatedly:

  // Set LEDs states to - all OFF
  digitalWrite(RED, LOW);     // Turn the red led off
  digitalWrite(BLUE, LOW);    // Turn the blue led off
  digitalWrite(GREEN, LOW);   // Turn the green led off
  delay(500);                 // Wait half a second

  // Set LEDs states to - RED
  digitalWrite(RED, HIGH);    // Turn the red led on
  digitalWrite(BLUE, LOW);    // Turn the blue led off
  digitalWrite(GREEN, LOW);   // Turn the green led off    
  delay(500);                 // Wait half a second

  // Set LEDs states to - BLUE
  digitalWrite(RED, LOW);     // Turn the red led off
  digitalWrite(BLUE, HIGH);   // Turn the blue led on
  digitalWrite(GREEN, LOW);   // Turn the green led off 
  delay(500);                 // Wait half a second

  // Set LEDs states to - GREEN
  digitalWrite(RED, LOW);     // Turn the red led off
  digitalWrite(BLUE, LOW);    // Turn the blue led off
  digitalWrite(GREEN, HIGH);  // Turn the green led on 
  delay(500);                 // Wait half a second

  // Set LEDs states to - RED AND GREEN
  digitalWrite(RED, HIGH);     // Turn the red led on
  digitalWrite(BLUE, LOW);    // Turn the blue led off
  digitalWrite(GREEN, HIGH);  // Turn the green led on 
  delay(500);                 // Wait half a second

  // Set LEDs states to - BLUE AND GREEN
  digitalWrite(RED, LOW);     // Turn the red led off
  digitalWrite(BLUE, HIGH);   // Turn the blue led on
  digitalWrite(GREEN, HIGH);  // Turn the green led on 
  delay(500);                 // Wait half a second

  // Set LEDs states to - RED AND BLUE
  digitalWrite(RED, HIGH);     // Turn the red led on
  digitalWrite(BLUE, HIGH);    // Turn the blue led on
  digitalWrite(GREEN, LOW);  // Turn the green led off
  delay(500);                 // Wait half a second

  // Set LEDs states to - RED, BLUE and GREEN
  digitalWrite(RED, HIGH);     // Turn the red led on
  digitalWrite(BLUE, HIGH);    // Turn the blue led on
  digitalWrite(GREEN, HIGH);  // Turn the green led on
  delay(500);                 // Wait half a second
  
}
