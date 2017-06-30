int out[] = {0, 1, 2, 3};
int in[] = {0, 1, 2, 3};
int sensorValue = 0;

void setup()
{
  pinMode(dOut, OUTPUT);
  Serial.begin(115200);
}

void loop()
{
    digitalWrite(dOut, HIGH);
    Serial.print(dOut);
    Serial.println(" Is now HIGH");
    delay(1);
    
    sensorValue = analogRead(aIn);
    Serial.println(sensorValue);
    delay(1);
    
    digitalWrite(dOut, LOW);
    Serial.print(dOut);
    Serial.println(" Is now LOW");
    delay(1);
    
    sensorValue = analogRead(aIn);
    Serial.println(sensorValue);
    delay(1);
      
}
