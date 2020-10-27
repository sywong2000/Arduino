#include <Tiny4kOLED.h>
#include "font8x16atari.h"
const DCfont *largeFont = FONT8X16ATARI;
#include <Wire.h>
 
 
void setup()
{
  Wire.begin();
  oled.begin();
  oled.on();
  oled.clear();
  oled.switchRenderFrame();
  oled.clear();
 
//  Serial.begin(115200);
//  while (!Serial);             // Leonardo: wait for serial monitor
//  Serial.println("\nI2C Scanner");
}
 
 
void loop()
{
  byte error, address;
  int nDevices;
 
//  Serial.println("Scanning...");
 
  nDevices = 0;
  String sAddr = "";
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    sAddr += String(address,HEX);
    sAddr += ",";
 
    if (error == 0)
    {
//      Serial.print("I2C device found at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.print(address,HEX);
//      Serial.println("  !");
      nDevices++;
      
    }
    else if (error==4)
    {
//      Serial.print("Unknown error at address 0x");
//      if (address<16)
//        Serial.print("0");
//      Serial.println(address,HEX);
    }    
  }
    oled.setFont(largeFont);
    oled.setCursor(32, 1);
    oled.print(nDevices);
//    oled.print(" ");
//    oled.print(sAddr);
    oled.switchFrame();
    delay(1000);

  
//  if (nDevices == 0)
//    Serial.println("No I2C devices found\n");
//  else
//    Serial.println("done\n");
 
//  delay(2000);           // wait 5 seconds for next scan
}
