/******************************************************************************
 * Example program I2C-ADC interface with Arduino.
 * 
 * SETUP:    I2C-ADC => Arduino
 * PIN7 => ground, PIN11 => A5(SCL), PIN12 => A4(SDA), PIN14 => +5V
 * Note:     The program is written for address 0x90 (Arduino address 0x48).
 * This program was tested using Arduino Nano
 * Document: AD7828 datasheet
 * Updated:  September 4, 2008
 * E-mail:   support@gravitech.us
 * Gravitech
 * (C) Copyright 2008 All Rights Reserved
 *******************************************************************************/

#include <Wire.h> 
#define numberOfChannels 8
#define numberOfChips 4

/*******************************************************************************
 * Setup
 *******************************************************************************/
void setup() 
{ 
  Serial.begin(115200);
  Wire.begin();             // join i2c bus (address optional for master) 
  delay(1000);
} 

/*******************************************************************************
 * Main Loop
 *******************************************************************************/
void loop() 
{
  // I2C write addresses
  const int I2C_address[4] = { 
    0x48, 0x49, 0x4A, 0x4B };  
  // Constant configuration data
  const byte DAT[8] = { 
    0x8C,0xCC,0x9C,0xDC,0xAC,0xEC,0xBC,0xFC };

  byte Adval_High, Adval_Low;            // Store A/D value (high byte, low byte)
  byte i, chipAddress;                   // Counter

  for (chipAddress = 0; chipAddress < numberOfChips; chipAddress++)
  {
    for (i=0; i < numberOfChannels; i++)
    {
      Wire.beginTransmission(I2C_address[chipAddress]);
      Wire.write(DAT[i]);                // Configure the device to read each CH  
      Wire.endTransmission(); 

      // Read A/D value
      Wire.requestFrom(I2C_address[chipAddress], 2);
      while(Wire.available())           // Checkf for data from slave
      { 
        Adval_High = Wire.read();       // Receive A/D high byte
        Adval_Low = Wire.read();        // Receive A/D low byte
      } 

      int adcVal = (Adval_High<<8) + Adval_Low;
      Serial.print(adcVal, DEC);
      if (i < numberOfChannels -1) {
        Serial.print(",");
      }
    }
    if (chipAddress < numberOfChips -1) {
      Serial.print(",");
    }
  }
  Serial.print('\n');
}




































