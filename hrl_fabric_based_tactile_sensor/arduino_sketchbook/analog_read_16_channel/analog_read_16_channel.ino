/*
  Analog serial reader
  by Tom Igoe
  Language: Arduino/Wiring
 
  Reads several analog inputs and sends their values out.
  This application assumes you have analog sensors attached
  to analog inouts 0 through 5. At the simplest, you can 
  hook up five potentiometers and it'll work fine.
 
  Created 24 May 2006
*/
 
// define the total number of analog sensors
// that you want to read
#define numberOfSensors 16
 
void setup() {
  // initialize the serial port:
  Serial.begin(115200);
  analogReference(EXTERNAL);
}

void loop() {
  // loop over the sensors:
  for (int thisSensor = 0; thisSensor < numberOfSensors; thisSensor++) {
    // read each sensor
    int sensorReading = analogRead(thisSensor);
    // print its value out as an ASCII numeric string
    Serial.print(sensorReading, DEC);
    // if this isn't the last sensor to read,
    // then print a comma after it
    if (thisSensor < numberOfSensors -1) {
      Serial.print(",");
    }
  }
  Serial.print("\n");
}
