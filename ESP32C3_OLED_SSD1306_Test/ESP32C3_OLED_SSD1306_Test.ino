#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// Set I2C bus to use: Wire, Wire1, etc.
// scl-->gpio6  sda-->gpio5
#define WIRE Wire
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

void setup() {
  Wire.begin(5,6);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.clear();
  oled.ssd1306WriteCmd(SSD1306_SEGREMAP);
  oled.ssd1306WriteCmd(SSD1306_COMSCANINC);
  oled.setFont(Callibri15);
  // starts at x=30, y=14
  oled.setRow(2);
  oled.setCol(30);
  oled.println("READY");
  Serial.begin(9600);
  Serial.println("READY");
}


void loop() {

}