#include <Arduino.h>
#include <Wire.h>

//I2C communication
//only 2 wires needed for comminication
// wires needed Serial Data (SDA) and Serial Clock (SCL)
// 


void setup() 
{
  Wire.begin();
  Serial.begin(9600);
}

void loop() 
{
  byte num=0;
  
  // set the 24C256 eeprom address to 0
  Wire.beginTransmission(80);
  Wire.write(0);  // address high byte
  Wire.write(0);  // address low byte
  Wire.endTransmission();
  
  // read 1 byte, from address 0
  Wire.requestFrom(80, 1);
  while(Wire.available()) {
    num = Wire.read();
  }
  Serial.print("num = ");
  Serial.println(num, DEC);
  
  // increment num
  num = num + 1;
  
  // write "num" to 24C256 eeprom at address zero
  Wire.beginTransmission(80);
  Wire.write(0);    // address high byte
  Wire.write(0);    // address low byte
  Wire.write(num);  // any more send starts writing
  Wire.endTransmission();
  
  // next time loop runs, it should retrieve the
  // same number it wrote last time... even if you
  // shut off the power
  delay(5000);
}


