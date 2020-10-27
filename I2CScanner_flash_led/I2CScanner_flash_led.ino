#include <Wire.h>
 
 
void setup()
{
  Wire.begin();
 
//  Serial.begin(115200);
//  while (!Serial);             // Leonardo: wait for serial monitor
//  Serial.println("\nI2C Scanner");
  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH);   // turn the LED on (HIGH is the voltage level)
}
 
 
void loop()
{
  byte error, address;
  int nDevices;
 
//  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
//      Serial.print("I2C device found at address 0x");
//      if (address<16)
//    {
//        Serial.print("0");
//      Serial.print(address,HEX);
//      Serial.println("  !");
      nDevices++;
//    }
    }
    else if (error==4)
    {
//      Serial.print("Unknown error at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
  {
        digitalWrite(1, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
        else
        {
          for (int n=0;n<nDevices;n++)
          {
            digitalWrite(1, HIGH);   // turn the LED on (HIGH is the voltage level)
            delay(50);                       // wait for a second
            digitalWrite(1, LOW);   // turn the LED on (HIGH is the voltage level)
            delay(50);                       // wait for a second
          }
        }

//    Serial.println("No I2C devices found\n");
//  else
//    Serial.println("done\n");
 
  delay(2000);           // wait 5 seconds for next scan
}
