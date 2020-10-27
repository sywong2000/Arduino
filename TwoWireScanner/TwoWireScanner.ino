#include <Wire.h>


TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
  byte error, address;
  int nDevices;

void setup()
{
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
  Serial.println("Scanning...");
  I2Cone.begin();
  I2Ctwo.begin();
}


void loop()
{
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    I2Cone.beginTransmission(address);
    error = I2Cone.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C [bus 0] device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }

  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    I2Ctwo.beginTransmission(address);
    error = I2Ctwo.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C [bus 0] device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }

  
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

}
