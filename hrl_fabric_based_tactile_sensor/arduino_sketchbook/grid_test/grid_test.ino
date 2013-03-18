int out[] = {2, 3, 4, 5};
int in[] = {3, 2, 1, 0};
int sensorValue = 0;

void setup()
{
  for(int outIndex = 0; outIndex < 4; outIndex++){
    pinMode(out[outIndex], OUTPUT);
  }
  Serial.begin(115200);
}

void loop()
{
  for(int outIndex = 0; outIndex < 4; outIndex++){
    digitalWrite(out[outIndex], HIGH);
    for(int inIndex = 0; inIndex < 4; inIndex++){
      sensorValue = analogRead(in[inIndex]);
      Serial.print(sensorValue);
      Serial.print("\t");
    }
    Serial.println();
    digitalWrite(out[outIndex], LOW);
  }
  
  Serial.println();
  delay(300);
}
