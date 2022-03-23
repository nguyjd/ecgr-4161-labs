const int TRIGPIN = 32;
const int ECHOPIN = 33;

#define SAMPLES_PER_READING 5
#define SENSOR_TIMEOUT 100000
#define MAX_DIST 400

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

    delay(200);

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

void setup() {
  // put your setup code here, to run once:

  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

  Serial.begin(9600);
  delay(1000);
  Serial.println("Starting HC-SR04 Ultrasonic Sensor Test.");

}

void loop() {
  // put your main code here, to run repeatedly:
  float pulseLength, centimeters;
  centimeters = ReadUltrasonic();
  
  delay(1000);

  if (centimeters != 0 && centimeters <= 400)
  {

    Serial.print("Distance = ");
    Serial.print(centimeters);
    Serial.println(" cm");
    
  }
  
  
  
}
